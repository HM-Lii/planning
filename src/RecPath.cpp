#include <ros/ros.h>
#include <rosbag/bag.h>
#include <ros/spinner.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <mutex>

using namespace std;
using namespace GeographicLib;

class PathRecorder {
 public:
  PathRecorder(ros::NodeHandle& nh)
      : count(0),
        initialized_(false),
        state_(State::INIT),
        earth_model_(Constants::WGS84_a(), Constants::WGS84_f()) {
    path_in_xy_.header.frame_id = "camera_init";
    path_pub_ = nh.advertise<nav_msgs::Path>("/ref_path", 1);
    fix_sub_ = nh.subscribe("/fix", 1, &PathRecorder::fixCallback, this);
  }
  void run(ros::NodeHandle& nh) {
    ros::Rate rate(2);
    ROS_INFO("Start listening keyboard.");
    while (ros::ok()) {

      if (kbhit()) {
        handleKeyboardInput();
      }
      switch (state_) {
        case State::INIT:
          break;
        case State::RECORD:
          recordFix();
          break;
        case State::WRITE_GO:
          WriteGoPath();
          break;
        case State::WRITE_WORK:
          WriteWorkPath();
          break;
        case State::STOP_WRITE:
          path_in_fix_.clear();
          path_in_xy_.poses.clear();
          state_ = State::INIT;
          initialized_ = false;
          break;
        default:
          break;
      }
      ros::spinOnce();
      rate.sleep();
    }
  }

 private:
  enum class State { INIT, RECORD, WRITE_WORK, WRITE_GO, STOP_WRITE };
  void handleKeyboardInput() {
    char c = getchar();
    if (c == ' ') {
      if (!initialized_) {
        ROS_WARN("Not initialized yet!");
        return;
      }
      state_ = State::RECORD;
      ROS_INFO("Start recorded.");
    } else if (c == 'g') {
      if (!initialized_) {
        ROS_WARN("Not initialized yet!");
        return;
      }
      state_ = State::WRITE_GO;
      ROS_INFO("Write go path bag...");
    } else if (c == 'w') {
      if (!initialized_) {
        ROS_WARN("Not initialized yet!");
        return;
      }
      state_ = State::WRITE_WORK;
      ROS_INFO("Write work path bag...");
    } else if (c == 'q') {
      ROS_INFO("Quit.");
      ros::shutdown();
    } else {
      ROS_INFO("Invalid input.");
    }
  }

  void recordFix() {
    lock_guard<mutex> lock(mtx_);
    geometry_msgs::PoseStamped current_point = Gps_Pt(current_fix_);
    if (pow(current_point.pose.position.x - latest_point_.pose.position.x, 2) +
            pow(current_point.pose.position.y - latest_point_.pose.position.y,
                2) <
        0.04) {
      ROS_WARN("too close the last fix! wait for next fix...");
      return;
    }
    ROS_INFO("a fix has been recorded!");
    path_in_fix_.push_back(current_fix_);
    path_in_xy_.poses.push_back(latest_point_);
    latest_point_ = current_point;
  }
  void clearFixs() {
    lock_guard<mutex> lock(mtx_);
    path_in_fix_.clear();
  }

  geometry_msgs::PoseStamped Gps_Pt(sensor_msgs::NavSatFix thisFix) {
    double x, y, z;
    local_projector_.Forward(thisFix.latitude, thisFix.longitude,
                             thisFix.altitude, x, y, z);

    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(0, 0, 0));
    tf2::Quaternion quaternion_;
    quaternion_.setRPY(0, 0, enu_heading_ * M_PI / 180.0);
    transform.setRotation(quaternion_);

    tf2::Vector3 body_offset(body_dx_, body_dy_, 0.0);
    tf2::Vector3 enu_offset = transform * body_offset;

    x += enu_offset.getX();
    y += enu_offset.getY();
    geometry_msgs::PoseStamped res;
    res.header.stamp = ros::Time::now();
    res.header.frame_id = "camera_init";
    res.pose.position.x = x;
    res.pose.position.y = y;
    res.pose.position.z = 0;
    return res;
  }
  void WriteGoPath() {
    rosbag::Bag bag;
    bag.open(
        "/home/li/Desktop/demo_ws/0528_ws/src/planning/path_bag/go_path.bag",
        rosbag::bagmode::Write);
    for (const auto& fix : path_in_fix_) {
      bag.write("fixes", ros::Time::now(), fix);
    }
    bag.close();
    ROS_INFO("go_path bag is writted.");
    state_ = State::STOP_WRITE;
  }
  void WriteWorkPath() {
    rosbag::Bag bag;
    bag.open(
        "/home/li/Desktop/demo_ws/0528_ws/src/planning/path_bag/wrok_path.bag",
        rosbag::bagmode::Write);
    for (const auto& fix : path_in_fix_) {
      bag.write("fixes", ros::Time::now(), fix);
    }
    bag.close();
    ROS_INFO("work_path bag is writted.");
    state_ = State::STOP_WRITE;
  }
  void fixCallback(const sensor_msgs::NavSatFixConstPtr& msg) {
    if (!msg) {
      ROS_ERROR("Received empty fix message");
      return;
    }
    if (msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
      ROS_ERROR("Received float gps message,please modify the gps signal!");
      return;
    }
    if ((!initialized_) && (count != 10)) {
      ROS_INFO("initializing...");
      count++;
      return;
    } else {
      latest_point_ = Gps_Pt(current_fix_);
      initialized_ = true;
    }
    current_fix_ = *msg;
  }
  int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
      ungetc(ch, stdin);
      return 1;
    }

    return 0;
  }

  Geocentric earth_model_;          // 地球模型
  LocalCartesian local_projector_;  // 本地投影器
  ros::Subscriber fix_sub_;
  ros::Publisher path_pub_;
  sensor_msgs::NavSatFix current_fix_;
  geometry_msgs::PoseStamped latest_point_;
  vector<sensor_msgs::NavSatFix> path_in_fix_;
  nav_msgs::Path path_in_xy_;
  double gps_heading_, enu_heading_, latitude_, longitude_, body_dx_, body_dy_;
  bool initialized_;  // 是否为第一次定位
  mutex mtx_;         // 创建一个锁
  State state_;
  int count ;
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "RecPath");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);  // 使用2个线程
  spinner.start();
  PathRecorder pr(nh);
  pr.run(nh);
  spinner.stop();
  return 0;
}
