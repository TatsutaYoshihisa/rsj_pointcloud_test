#include <ros/ros.h> 
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>  
#include <pcl/kdtree/kdtree.h>  
#include <pcl/segmentation/extract_clusters.h>  
#include <pcl/segmentation/sac_segmentation.h> 
#include <pcl/filters/extract_indices.h> 

//pcl::PointCloud<PointXYZ> をPointCloudに名前を変えて使うよ
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
//距離
const float TARGET_DISTANCE = 1.0; 
const float DISTANCE_THRESHOLD  = 0.02 ;
//マーカー作成関数の定義
visualization_msgs::Marker makeMarker(const std::string& frame_id,const std::string& ns,int id ,const Eigen::Vector4f& min_pt,const Eigen::Vector4f& max_pt ,float r,float g,float b,float a){
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = (max_pt.x() + min_pt.x())/2;
    marker.pose.position.y = (max_pt.y() + min_pt.y())/2;
    marker.pose.position.z = (max_pt.z() + min_pt.z())/2;

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    marker.scale.x = max_pt.x() - min_pt.x();
    marker.scale.y = max_pt.y()- min_pt.y();
    marker.scale.z = max_pt.z() - min_pt.z();

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    marker.lifetime = ros::Duration(0.1);
    
    return marker;

}

class RsjPointCloudTestNode
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber sub_points_;  //点群購読用のサブスクライバー

    //フィルターのPassThroughのインスタンスを追加
    pcl::PassThrough<PointT> pass_;
    PointCloud::Ptr cloud_passthrough_;
    ros::Publisher pub_passthrough_;

    //ボクセル化してダウンサンプリングするフィルター、VoxelGridのインスタンスを追加
    pcl::VoxelGrid<PointT> voxel_;
    PointCloud::Ptr cloud_voxel_;
    ros::Publisher pub_voxel_;

    
    //平面推定
    const int MAX_PLANES = 15; // 検出する平面の最大数
    const int MIN_PLANE_POINTS = 50; // 有効な平面とみなす最小点数
    //平面方程式と平面を抽出された点のインデックス
    pcl::ModelCoefficients::Ptr coefficients_;
    pcl::PointIndices::Ptr inliers_;
    //RANSACによる検出．S
    pcl::SACSegmentation<PointT> seg_;
    PointCloud::Ptr cloud_plane_;
    pcl::ExtractIndices<PointT> extract_;
    ros::Publisher pub_plane_;
    ros::Publisher pub_plane_result_;
    ros::Publisher pub_planes_markers_;

    //クラスターのフィルターclusterのインスタンスを追加
    pcl::search::KdTree<PointT>::Ptr tree_;
    pcl::EuclideanClusterExtraction<PointT> ec_;
    ros::Publisher pub_clusters_;


public:
    RsjPointCloudTestNode()
    :nh_()
    ,pnh_("~")
    {
        //サブスクライバの設定
        sub_points_ = nh_.subscribe("velodyne_points",1,&RsjPointCloudTestNode::cbPoints,this);
        
        //高さフィルターとパブリッシャーの設定
        pass_.setFilterFieldName("z");//z軸（高さ）の値でフィルタをかける
        pass_.setFilterLimits(-1.0, 1.0);//0.1~1.0mの間にある点群を抽出
        cloud_passthrough_.reset(new PointCloud());
        pub_passthrough_ = nh_.advertise<sensor_msgs::PointCloud2>("passthrough", 1);

        //Voxelフィルターとパブリッシャーの設定
        //0.025f*0.025f*0.025fの立方体の重心を計算し、その点を新しく点群としている
        voxel_.setLeafSize(0.015f, 0.015f, 0.015f);
        cloud_voxel_.reset(new PointCloud());
        pub_voxel_ = nh_.advertise<sensor_msgs::PointCloud2>("voxel", 1);

        // 平面推定の初期化
        coefficients_.reset(new pcl::ModelCoefficients);
        inliers_.reset(new pcl::PointIndices);
        cloud_plane_.reset(new PointCloud());

        // 平面検出の設定
        seg_.setOptimizeCoefficients(true);
        seg_.setModelType(pcl::SACMODEL_PLANE);
        seg_.setMethodType(pcl::SAC_RANSAC);//検知方法
        seg_.setMaxIterations(100);
        seg_.setDistanceThreshold(0.02);
        pub_plane_ = nh_.advertise<sensor_msgs::PointCloud2>("plane", 1);
        pub_plane_result_ = nh_.advertise<sensor_msgs::PointCloud2>("plane_result", 1);
        pub_planes_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("plane_markers", 1);

        //clusterフィルターとパブリッシャーの設定
        tree_.reset(new pcl::search::KdTree<PointT>());
        ec_.setClusterTolerance(0.15);//同じクラスタに属するとみなす点の最大距離
        ec_.setMinClusterSize(100);//有効なクラスタとみなすための最小点数
        ec_.setMaxClusterSize(5000);//有効なクラスタとみなすための最大点数
        ec_.setSearchMethod(tree_); // KdTreeなどの検索方法を指定
        pub_clusters_ = nh_.advertise<visualization_msgs::MarkerArray>("clusters", 1);

    }

    void cbPoints(const sensor_msgs::PointCloud2::ConstPtr &msg){
        try{
            //ROSメッセージからPCLクラウドに変換
            PointCloud::Ptr cloud_src(new PointCloud);
            pcl::fromROSMsg(*msg, *cloud_src);

            //高さフィルタ
            pass_.setInputCloud(cloud_src);
            pass_.filter(*cloud_passthrough_);

            //ボクセル化
            voxel_.setInputCloud(cloud_passthrough_);
            voxel_.filter(*cloud_voxel_);

            // 平面推定
            PointCloud::Ptr remaining_cloud(new PointCloud(*cloud_voxel_));
            visualization_msgs::MarkerArray plane_markers;
            float r =0.0f, g = 1.0f, b = 0.0f,a = 0.5f;
            //平面の数がMAX_PLANESを超えるまで繰り返す
            for (int plane_count = 0; plane_count < MAX_PLANES;plane_count++){

                // 平面検出
                seg_.setInputCloud(remaining_cloud);
                //inliers_は一番大きい平面とみなされた点のインデックス番号が格納される
                //coefficients_は平面方程式 ax + by + cz + d = 0のa,b,c,dがcoefficients->value[0]~[3]に格納される
                seg_.segment(*inliers_, *coefficients_);

                //一番大きい平面の点の数がMIN_PLANE_POINTSより少ない場合、これ以上平面は存在しないので平面推定は終了
                if(inliers_->indices.size() < MIN_PLANE_POINTS){
                    break;
                }
                //平面抽出後の点群を格納する用のポイントクラウドを宣言
                PointCloud::Ptr plane_cloud(new PointCloud());
                //この点群から平面を抽出する
                extract_.setInputCloud(remaining_cloud);
                //setInputしたCloudから抽出するための点のインデックス番号を入力する
                extract_.setIndices(inliers_);
                //remaining_cloudからinliers_の点群を抽出する(false:平面に属する点のみ抽出、true:平面に属する点のみ除去)
                extract_.setNegative(false);
                //抽出後の点群をplane_cloudに格納する
                extract_.filter(*plane_cloud); 

                //平面のマーカーを作成する
                Eigen::Vector4f min_pt, max_pt;
                //点群の最大値と最小値をもとめる
                pcl::getMinMax3D(*plane_cloud, min_pt, max_pt);
                //平面マーカーを追加する
                plane_markers.markers.push_back(
                    makeMarker(msg->header.frame_id,"plane",plane_count,min_pt,max_pt,r,g,b,a)
                );
                //残りの点群から抽出した平面を除去
                extract_.setInputCloud(remaining_cloud);
                extract_.setIndices(inliers_);
                extract_.setNegative(true); 
                
                PointCloud::Ptr temp_cloud(new PointCloud());
                extract_.filter(*temp_cloud);
                remaining_cloud = temp_cloud;
            }

            // 平面マーカーをパブリッシュ
            if (!plane_markers.markers.empty()) {
                pub_planes_markers_.publish(plane_markers);
            }

            //クラスタリング処理
            //点群データのクラスタリング処理と、それを可視化するマーカー生成を行う
            //各クラスターに属する点群の添字(Indice)をクラスターごとに格納する
            std::vector<pcl::PointIndices> cluster_indices;
            //ポインタ型からメンバ変数にアクセするときは->演算子を使用する
            //オブジェクトからメンバ変数にアクセスするときは.演算子を使用する
            tree_->setInputCloud(remaining_cloud);
            //ユークリッドクラスタ抽出アルゴリズムにボクセル化した点群を代入する
            ec_.setInputCloud(remaining_cloud);
            //クラスタリングを行って結果をcluster_indicesに格納する
            //このときクラスタリングした点のインデックス(点の番号)のみが抽出(extract)される
            ec_.extract(cluster_indices);

            //マーキング処理
            //RVizで可視化するためのマーカー配列を宣言する
            //複数のクラスタを一度に表示するために使用する
            visualization_msgs::MarkerArray marker_array;
            //各マーカーにIDを振り分ける
            //イテレータを使用して各要素にmarker_idを振り分ける
            int marker_id = 0;
            size_t ok ;
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(),
                it_end = cluster_indices.end();it != it_end; ++it, ++marker_id){
                //クラスターを囲む立方体の最大の点と最小の点のベクトル(x,y,z,w=1)を宣言する
                Eigen::Vector4f min_pt, max_pt;
                //インデックスitのcloud_voxel_クラスターを囲む立方体の最大の点と最小の点を計算
                pcl::getMinMax3D(*remaining_cloud, *it, min_pt, max_pt);
                //立方体のサイズを計算する
                Eigen::Vector4f cluster_size = max_pt - min_pt;
                

                if (cluster_size.x() > 0 && cluster_size.y() > 0 && cluster_size.z() > 0){
                    bool is_ok = true;
                    if (cluster_size.x() < 0.05 || cluster_size.x() > 0.9){
                        is_ok = false;
                    }else if (cluster_size.y() < 0.05 || cluster_size.y() > 0.9){
                        is_ok = false;
                    }else if (cluster_size.z() < 0.05 || cluster_size.z() > 0.9){
                        is_ok = false;
                    }

                    bool has_target_point = false;
                    
                    for (const auto& idx : it->indices) {
                        // 点の座標を取得
                        float x = remaining_cloud->points[idx].x;
                        float y = remaining_cloud->points[idx].y;
                        
                        // ライダー原点からの2D距離を計算
                        float distance = ::hypot(x, y);
                        
                        // 指定距離に一致する点があれば、フラグを立てて終了
                        if ( distance  < TARGET_DISTANCE + DISTANCE_THRESHOLD) {
                            has_target_point = true;
                            break;
                        }
                    }

                    r = 0.0f;
                    g = 1.0f;
                    b = 1.0f;
                    a = 0.5f;

                    if(is_ok){
                        g = 0.0f;
                        b = 1.0f;
                    }
                    if(has_target_point){
                        g = 0.0f;
                        r = 1.0f;
                    }


                    //サイズが０以上の有効な場合のみマーカーを作成しマーカー配列に追加する
                    //marker_array.markersベクトルの末尾にmakeMarkerで作成されるオブジェクトを追加する
                    marker_array.markers.push_back(
                        makeMarker(
                            msg->header.frame_id, "cluster", marker_id, min_pt, max_pt, r, g, b, a));
                }
            }
            //マーカー配列が空でない場合のみパブリッシュ
            if (!marker_array.markers.empty())
            {
                pub_clusters_.publish(marker_array);
            }

            //PCLクラウドからROSメッセージに変換
            sensor_msgs::PointCloud2 output_p,output_v,output_r;
            pcl::toROSMsg(*cloud_passthrough_, output_p);
            pcl::toROSMsg(*cloud_voxel_, output_v);
            pcl::toROSMsg(*remaining_cloud, output_r);
            output_p.header = msg->header;  // ヘッダー情報をコピー
            output_v.header = msg->header; 
            output_r.header = msg->header;
            //パブリッシュ
            pub_passthrough_.publish(output_p);
            pub_voxel_.publish(output_v);
            pub_plane_result_.publish(output_r);


            ROS_INFO("points (src: %u, paththrough: %zu, voxelgrid: %zu,"
             " cluster: %zu)",
             msg->width*msg->height, cloud_passthrough_->size(), cloud_voxel_->size(),
             cluster_indices.size());
        }
        catch(std::exception &e){
            ROS_ERROR("%s",e.what());
        }
    }

};

int main(int argc, char **argv)
{
    // ROSノードの初期化
    ros::init(argc, argv, "rsj_pointcloud_test_node_cluster_v2");
    
    // ノードのインスタンス作成
    RsjPointCloudTestNode node;
    
    // スピナーでコールバックを処理
    ros::spin();
    
    return 0;
}
