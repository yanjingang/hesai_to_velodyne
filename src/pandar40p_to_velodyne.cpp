#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>

// Velodyne DataType
struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

// Hesai DataType
struct HesaiPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    double timestamp;
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(HesaiPointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
    (uint16_t, ring, ring)
)

ros::Subscriber subHesaiCloud;  // Sub Hesai PointCloud2
ros::Publisher pubVPoints;      // Pub Velodyne PointCloud2
// ros::Publisher pubVPackets;     // Pub Velodyne Packets


template <typename T>
bool checkPointNan(T point) {
    // remove nan point, or the feature assocaion will crash, the surf point will containing nan points
    if (pcl_isnan(point.x) || pcl_isnan(point.y) || pcl_isnan(point.z)) {
        ROS_ERROR("checkPointNan: Containing nan point!");
        return true;
    } else {
        return false;
    }
}


void hesaiHandler(const sensor_msgs::PointCloud2 &currentMsg) {
    // Init
    pcl::PointCloud<HesaiPointXYZIRT>::Ptr hesaiCloud(new pcl::PointCloud<HesaiPointXYZIRT>());
    pcl::fromROSMsg(currentMsg, *hesaiCloud);
    pcl::PointCloud<VelodynePointXYZIRT>::Ptr velodyneCloud(new pcl::PointCloud<VelodynePointXYZIRT>());
    velodyneCloud->is_dense = currentMsg->is_dense;

    // Convert to Velodyne format
    double time_begin = hesaiCloud->points[0].timestamp;
    for (int i = 0; i < hesaiCloud->points.size(); ++i)
    {
        if (checkPointNan(hesaiCloud->points[i]))
            continue;

        HesaiPointXYZIRT &src = hesaiCloud->points[i];
        VelodynePointXYZIRT dst;
        dst.x = src.y * -1;
        dst.y = src.x;
        dst.z = src.z;
        dst.intensity = src.intensity;
        dst.ring = src.ring;                    // 激光点在竖直方向上所属的线束序号
        dst.time = src.timestamp - time_begin;  // 当前激光点相对于当前激光帧第一个激光点的扫描时间， 单位秒s
        velodyneCloud->points.push_back(dst);
        // ROS_INFO("dst.ring=" << dst.ring);
    }

    // Publish velodyne_points
    sensor_msgs::PointCloud2 velodyneCloudMsg;
    pcl::toROSMsg(*velodyneCloud, velodyneCloudMsg);
    velodyneCloudMsg.header = currentMsg.header;
    velodyneCloudMsg.header.frame_id = currentMsg.header.frame_id;
    pubVPoints.publish(velodyneCloudMsg);
    
    // Publish velodyne_packets
    // velodyne_msgs::VelodyneScan velodynePackets;
    // velodynePackets.header = currentMsg.header;
    // velodynePackets.packets = hesaiCloud->points.size();

}

int main(int argc, char **argv)
{
    ros::init(argc, argv);
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::NodeHandle nh;

    std::string liadr_topic;
    std::string pub_points_topic;
    std::string pub_packets_topic;
    ros::param::get("lidar_topic", liadr_topic); 
    ros::param::get("pub_points_topic", pub_points_topic);
    ros::param::get("pub_packets_topic", pub_packets_topic);

    subHesaiCloud = nh.subscribe(liadr_topic, 1, hesaiHandler);
    pubVPoints = nh.advertise<sensor_msgs::PointCloud2>(pub_points_topic, 1);
    // pubVPackets = nh.advertise<velodyne_msgs::VelodyneScan>(pub_packets_topic, 1);

    ROS_INFO("Listening to pandar40p points ......");
    ros::spin();
    return 0;
}
