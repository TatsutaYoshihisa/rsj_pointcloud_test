#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h> //rosのヘッダも必要
#include <pcl_conversions/pcl_conversions.h> // ROSメッセージとPCLの変換用
#include <sensor_msgs/PointCloud2.h>

//pcl::PointCloud<PointXYZ> をPointCloudに名前を変えて使うよ
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class RsjPointCloudTestNode
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber sub_points_;  //点群購読用のサブスクライバー
    pcl::PassThrough<PointT> pass_;
    PointCloud::Ptr cloud_passthrough_;
    ros::Publisher pub_passthrough_;

public:
    RsjPointCloudTestNode()
    :nh_()
    ,pnh_("~")
    {
        pass_.setFilterFieldName("z");//z軸（高さ）の値でフィルタをかける
        pass_.setFilterLimits(0.1, 1.0);//0.1~1.0mの間にある点群を抽出
        cloud_passthrough_.reset(new PointCloud());
        pub_passthrough_ = nh_.advertise<sensor_msgs::PointCloud2>("passthrough", 1);
        sub_points_ = nh_.subscribe("velodyne_points",1,&RsjPointCloudTestNode::cbPoints,this);
    }

    void cbPoints(const sensor_msgs::PointCloud2::ConstPtr &msg){
        try{
            //ROSメッセージからPCLクラウドに変換
            PointCloud::Ptr cloud_src(new PointCloud);
            pcl::fromROSMsg(*msg, *cloud_src);

            //ここにcloud_srcに対するフィルタ処理を書く
            pass_.setInputCloud(cloud_src);
            pass_.filter(*cloud_passthrough_);

            //PCLクラウドからROSメッセージに変換
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*cloud_passthrough_, output);
            output.header = msg->header;  // ヘッダー情報をコピー

            //パブリッシュ
            pub_passthrough_.publish(output);
            ROS_INFO("points (src: %zu paththrough %zu)",cloud_src->size(),cloud_passthrough_->size());
        }
        catch(std::exception &e){
            ROS_ERROR("%s",e.what());
        }
    }

};

int main(int argc, char **argv)
{
    // ROSノードの初期化
    ros::init(argc, argv, "rsj_pointcloud_test_node");
    
    // ノードのインスタンス作成
    RsjPointCloudTestNode node;
    
    // スピナーでコールバックを処理
    ros::spin();
    
    return 0;
}
