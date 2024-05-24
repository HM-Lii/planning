#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include  <std_msgs/Int8.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <Eigen/Eigen>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
using namespace std;
using namespace geometry_msgs;
using namespace GeographicLib;

class GpsPath   {
 public:
  GpsPath(ros::NodeHandle& nh) : cmd_type(CmdType::STOP) {
    Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());
    go_path_pub_ = nh.advertise<nav_msgs::Path>("/ref_path", 1);
    fixSub_ = nh.subscribe("/fix", 1, &GpsPath::fixCallback, this);
    headingSub_ = nh.subscribe("/heading", 1, &GpsPath::headingCallback, this);
    ros::Subscriber cmd_go_sub = nh.subscribe<std_msgs::Int8>(
        "/cmd_go", 100, &GpsPath::CmdGoCallback, this);
    ros::Subscriber cmd_work_sub = nh.subscribe<std_msgs::Int8>(
        "/cmd_work", 100, &GpsPath::CmdWorkCallback, this);
    rosbag::Bag go_bag,work_bag;
    go_bag.open(
        "/home/li/Desktop/demo_ws/0528_ws/src/planning/path_bag/go_path.bag",
        rosbag::bagmode::Read);
    work_bag.open(
        "/home/li/Desktop/demo_ws/0528_ws/src/planning/path_bag/work_path.bag",
        rosbag::bagmode::Read);
    // bag.open(
    //     "${workspaceFolder}/src/planning/go_path_bag/ref_path.bag",
    //     rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("fixes"));

    rosbag::View view_go(go_bag, rosbag::TopicQuery(topics));
    rosbag::View view_work(work_bag, rosbag::TopicQuery(topics));

    foreach (rosbag::MessageInstance const m, view_go) {
      sensor_msgs::NavSatFix::ConstPtr fix =
          m.instantiate<sensor_msgs::NavSatFix>();
      if (fix != NULL) go_path_in_fix_.push_back(*fix);
    }
    foreach (rosbag::MessageInstance const m, view_work) {
      sensor_msgs::NavSatFix::ConstPtr fix =
          m.instantiate<sensor_msgs::NavSatFix>();
      if (fix != NULL) work_path_in_fix_.push_back(*fix);
    }
    go_bag.close();
    work_bag.close();
    ROS_INFO("Read %i fixes of go_path.", go_path_in_fix_.size());
    ROS_INFO("Read %i fixes of work_path.", work_path_in_fix_.size());
    ros::Rate rate(1);
    while(!initialized_){
      ros::spinOnce();
      rate.sleep();
    }
    GpsPtGo();
    GpsPtWork();
  }
  nav_msgs::Path GetPath(){
    return go_path_;
  }
  void PublishGoPath(){
    go_path_pub_.publish(go_path_);
    ROS_INFO("Publish go_path.");
  }
  void PublishWorkPath(){
    go_path_pub_.publish(work_path_);
    
  }
  void CmdGoCallback(const std_msgs::Int8::ConstPtr& msg) {
  ROS_INFO("cmd_go received");
  if (msg->data==1) {
    ROS_INFO("cmd_go received");
    cmd_type=CmdType::GO;
  }
}

void CmdWorkCallback(const std_msgs::Int8::ConstPtr& msg) {
  ROS_INFO("cmd_work received");
  if (msg->data==1) {
    ROS_INFO("cmd_work received");
    cmd_type=CmdType::WORK;
  }
}
enum class CmdType {
  STOP,
  GO ,
  WORK ,
};
CmdType GetCmdType() { return cmd_type; }

protected:
void GpsPtGo() {
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(0, 0, 0));
  tf2::Quaternion quaternion_;
  quaternion_.setRPY(0, 0, (heading_ - 90) * M_PI / 180.0);
  transform.setRotation(quaternion_);

  for (auto fix : go_path_in_fix_) {
    PoseStamped pt_temp;
    double temp_x, temp_y;
    local_projector_.Forward(fix.latitude, fix.longitude, fix.altitude, temp_x,
                             temp_y, pt_temp.pose.position.z);
    tf2::Vector3 pt_init(temp_x, temp_y, 0.0);
    tf2::Vector3 pt = transform * pt_init;
    pt_temp.pose.position.x = pt.getX();
    pt_temp.pose.position.y = pt.getY();
    go_path_.poses.push_back(pt_temp);
  }
  ROS_INFO("go_path size: %i", go_path_.poses.size());
  go_path_.header.frame_id = "camera_init";
  go_path_.header.stamp = ros::Time::now();
  }
    void GpsPtWork() {
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(0, 0, 0));
    tf2::Quaternion quaternion_;
    quaternion_.setRPY(0, 0, (heading_ - 90) * M_PI / 180.0);
    transform.setRotation(quaternion_);

    for (auto fix : work_path_in_fix_) {
      PoseStamped pt_temp;
      double temp_x, temp_y;
      local_projector_.Forward(fix.latitude, fix.longitude, fix.altitude, temp_x, temp_y, pt_temp.pose.position.z);
      tf2::Vector3 pt_init(temp_x, temp_y, 0.0);
      tf2::Vector3 pt = transform * pt_init;
      pt_temp.pose.position.x = pt.getX();
      pt_temp.pose.position.y = pt.getY();
      work_path_.poses.push_back(pt_temp);
    }
    ROS_INFO("work_path size: %i", work_path_.poses.size());
    work_path_.header.frame_id = "camera_init";
    work_path_.header.stamp = ros::Time::now();
  }
  void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    if (msg->status.status == sensor_msgs::NavSatStatus::STATUS_GBAS_FIX) {
      if ((!initialized_) && (count != 10)) {
        // 执行初始化操作
        local_projector_.Reset(msg->latitude, msg->longitude, msg->altitude);
        // 设置初始化完成标志
        count++;
        ROS_INFO("Record the %i st fix and heading.", count);
      } else {
        ROS_INFO("Record the final  fix  and  heading.");
        initialized_ = true;
        count = 0;
        fixSub_.shutdown();
      }
    }
  }

  void headingCallback(const Pose2D& msg) {
            heading_ = 90 - msg.theta;
        if (heading_ < 0) {
            heading_ += 360;
        }
  }

  bool check(nav_msgs::Path& path){
    return true;
  }
  vector<sensor_msgs::NavSatFix> go_path_in_fix_,work_path_in_fix_;
  double heading_;
  vector<Pose> poses_;
  nav_msgs::Path go_path_,work_path_;
  LocalCartesian local_projector_;
  ros::Publisher go_path_pub_;   
  ros::Subscriber fixSub_;
  ros::Subscriber headingSub_;
  bool initialized_=false;
  int count=0; 
  CmdType cmd_type;
};
