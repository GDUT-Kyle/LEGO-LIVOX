#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver/CustomMsg.h"
#include "lego_livox/common.h"

Eigen::Affine3f Ext_Livox = Eigen::Affine3f::Identity();

ros::Publisher pub_pcl_out0, pub_pcl_out1;
uint64_t TO_MERGE_CNT = 1; 
constexpr bool b_dbg_line = false;
// 创建一个循环队列用于存储雷达帧
std::vector<livox_ros_driver::CustomMsgConstPtr> livox_data;
void LivoxMsgCbk1(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in) {
  livox_data.push_back(livox_msg_in);
  // 第一帧则跳过
  if (livox_data.size() < TO_MERGE_CNT) return;

  pcl::PointCloud<PointType> pcl_in;

  // 遍历队列
  for (size_t j = 0; j < livox_data.size(); j++) {
    // 通过引用，方便操作每一帧
    auto& livox_msg = livox_data[j];
    // 获取该帧最后一个点的相对时间
    auto time_end = livox_msg->points.back().offset_time;
    // 重新组织成PCL的点云
    for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
      PointType pt;
      pt.x = livox_msg->points[i].x;
      pt.y = livox_msg->points[i].y;
      pt.z = livox_msg->points[i].z;
//      if (pt.z < -0.3) continue; // delete some outliers (our Horizon's assembly height is 0.3 meters)
      float s = livox_msg->points[i].offset_time / (float)time_end;
//       ROS_INFO("_s-------- %.6f ",s);
      // 线数——整数，时间偏移——小数
      pt.intensity = livox_msg->points[i].line + s*0.1; // The integer part is line number and the decimal part is timestamp
//      ROS_INFO("intensity-------- %.6f ",pt.intensity);
      pt.curvature = livox_msg->points[i].reflectivity * 0.001;
      // ROS_INFO("pt.curvature-------- %.3f ",pt.curvature);
      pcl_in.push_back(pt);
    }
  }

  /// timebase 5ms ~ 50000000, so 10 ~ 1ns
  pcl::transformPointCloud(pcl_in, pcl_in, Ext_Livox);

  // 最新一帧的时间戳
  unsigned long timebase_ns = livox_data[0]->timebase;
  ros::Time timestamp;
  timestamp.fromNSec(timebase_ns);

  //   ROS_INFO("livox1 republish %u points at time %f buf size %ld",
  //   pcl_in.size(),
  //           timestamp.toSec(), livox_data.size());

  sensor_msgs::PointCloud2 pcl_ros_msg;
  pcl::toROSMsg(pcl_in, pcl_ros_msg);
  pcl_ros_msg.header.stamp.fromNSec(timebase_ns);
  pcl_ros_msg.header.frame_id = "/livox";
  pub_pcl_out1.publish(pcl_ros_msg);
  livox_data.clear();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_repub");
  ros::NodeHandle nh;

  ROS_INFO("start livox_repub");

  Eigen::Vector3f Ext_trans(ext_livox[0], ext_livox[1], ext_livox[2]);
  Eigen::AngleAxisf rollAngle(ext_livox[3], Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(ext_livox[4], Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yawAngle(ext_livox[5], Eigen::Vector3f::UnitZ()); 
  Eigen::Quaternionf quaternion;
  quaternion=yawAngle*pitchAngle*rollAngle;
  Ext_Livox.pretranslate(Ext_trans);
  Ext_Livox.rotate(quaternion);

  ros::Subscriber sub_livox_msg1 = nh.subscribe<livox_ros_driver::CustomMsg>(
      "/livox/lidar", 100, LivoxMsgCbk1);
  pub_pcl_out1 = nh.advertise<sensor_msgs::PointCloud2>("/livox_pcl0", 100);

  ros::spin();
}
