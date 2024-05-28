#include <planning/pure_persuit.h>

using namespace std;

PurePersuit::PurePersuit(ros::NodeHandle& nh_) : enableObstacleAvoidance(false){
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 20);
  preview_pose_pub_= nh_.advertise<geometry_msgs::PoseStamped>("/preview_pose", 20);
  if(enable_gps){
    heading_sub_ = nh_.subscribe("/heading", 20, &PurePersuit::headingCallback, this);
  }
  pose_sub_ =
      nh_.subscribe("/Odometry", 20, &PurePersuit::poseCallback, this);
  path_sub_ = nh_.subscribe("ref_path", 20, &PurePersuit::pathCallback, this);
  cout << "listening to odometry!!" << endl;
  nh_.param("/pure_persuit/preview_distance", preview_distance_, 2.0);
  nh_.param("/pure_persuit/unit_length", unit_length_, 0.1);
  nh_.param("/pure_persuit/velocity", vel_, 0.3);
}
void PurePersuit::poseCallback(const nav_msgs::Odometry::ConstPtr& currentWaypoint) {
  if (ref_path.poses.empty()) {
    // 如果参考路径为空，则无法计算预瞄点
    cout << "warning: ref_path is empty!!" << endl;
    return;
  }
  if (if_reach) {
    cout << "warning: already reach the goal!!" << endl;
    return;
  }
  auto currentPoint = currentWaypoint->pose.pose;
  geometry_msgs::Pose nearestPoint;
  geometry_msgs::PoseStamped previewPoint;
  findNearestAndPreviewPoints(currentPoint, nearestPoint, previewPoint);
  double heading_goal =
      atan2((previewPoint.pose.position.y - currentPoint.position.y),
            (previewPoint.pose.position.x - currentPoint.position.x));
  if (enable_gps) {
    current_heading_ = 90- current_heading_;
    if(current_heading_<=-180)
      current_heading_+=360;
    cout<<"current_heading after transform: "<<current_heading_;

    current_heading_ = current_heading_ * M_PI / 180;
  } else {
    current_heading_ = tf::getYaw(currentPoint.orientation);
  }
  float delta_angle = calculateDeltaAngle(heading_goal,current_heading_);
  // 当前点和目标点的距离Id
  cout << "; heading_goal: " << heading_goal;
  cout << "; delta_angle: " << delta_angle << endl;
  // cout<<"previewPoint: "<<previewPoint.pose.position.x<<","<<previewPoint.pose.position.y<<endl;
  // cout<<"currentPoint: "<<currentPoint.position.x<<","<<currentPoint.position.y<<endl;
  float dis_cur_pre =
      sqrt(pow(previewPoint.pose.position.y - nearestPoint.position.y, 2) +
           pow(previewPoint.pose.position.x - nearestPoint.position.x, 2));
  // 发布运动指令:线速度和角速度
  if (dis_cur_pre > SLOW_DOWN_DISTANCE) {
    if (fabs(delta_angle) < ANGLE_THRESHOLD) {// 如果分母为0或接近于0，则无法计算变量R的值
      ROS_INFO("delta_angle is too small, can't calculate R");
      publishVelocityMessage(vel_, 0);// 此时直线行驶
    } else {
      float R = dis_cur_pre / (2 * sin(delta_angle));
      publishVelocityMessage(vel_, vel_ / R);
    }
    if_reach = false;
  } else if (dis_cur_pre > STOP_DISTANCE) {
    float R = dis_cur_pre / (2 * sin(delta_angle));
    publishVelocityMessage(vel_ / 2, vel_ / R);// 减速
    cout << "close to the end, slowing down!!" << endl;
    if_reach = false;
  } else {
    publishVelocityMessage(0, 0);// 停止
    cout << "reach the goal!!" << endl;
  }
}
void PurePersuit::headingCallback(const geometry_msgs::Pose2D::ConstPtr& currentHeading) {
  current_heading_ = currentHeading->theta;
  ROS_INFO("current_heading: %f", current_heading_);
} 

void PurePersuit::publishVelocityMessage(float linear, float angular) {
  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x = linear;
  vel_msg.angular.z = angular;
  vel_pub_.publish(vel_msg);
}
void PurePersuit::findNearestAndPreviewPoints(
    const geometry_msgs::Pose &currentPoint, geometry_msgs::Pose &nearestPoint,
    geometry_msgs::PoseStamped &previewPoint) {
  int nearestIndex;
  double nearestDis = std::numeric_limits<double>::max();
  for (int i = 0; i < pointNum; i++) {
    double dx = ref_path.poses[i].pose.position.x - currentPoint.position.x;
    double dy = ref_path.poses[i].pose.position.y - currentPoint.position.y;
    double dis = dx * dx + dy * dy;
    if (dis < nearestDis) {
      nearestDis = dis;
      nearestPoint = ref_path.poses[i].pose;
      nearestIndex = i;
    }
  }
  int preview_num = preview_distance_ / unit_length_;
  int preview_index = std::min(nearestIndex + preview_num, pointNum - 1);
  previewPoint.header.frame_id = "map";
  previewPoint.header.stamp = ros::Time::now();
  previewPoint.pose = ref_path.poses[preview_index].pose;
  preview_pose_pub_.publish(previewPoint);
}
void PurePersuit::findNearestAndPreviewPoints_(
    const geometry_msgs::Pose &currentPoint, geometry_msgs::Pose &nearestPoint,
    geometry_msgs::PoseStamped &previewPoint) {
  int nearestIndex = lastNearestIndex;
  double nearestDis = std::numeric_limits<double>::max();

  // Start searching from the last nearest index
  for (int i = lastNearestIndex; i < pointNum; i++) {
    double dx = ref_path.poses[i].pose.position.x - currentPoint.position.x;
    double dy = ref_path.poses[i].pose.position.y - currentPoint.position.y;
    double dis = dx * dx + dy * dy;
    if (dis < nearestDis) {
      nearestDis = dis;
      nearestPoint = ref_path.poses[i].pose;
      nearestIndex = i;
    }
  }
  lastNearestIndex = nearestIndex;  // Update the last nearest index

  int preview_num = preview_distance_ / unit_length_;
  int preview_index = std::min(nearestIndex + preview_num, pointNum - 1);
  previewPoint.header.frame_id = "map";
  previewPoint.header.stamp = ros::Time::now();
  if (preview_index < pointNum) {
    previewPoint.pose = ref_path.poses[preview_index].pose;
  }
  preview_pose_pub_.publish(previewPoint);
}
float PurePersuit::calculateDeltaAngle(float a, float b) {
  float d1, d2;
  d1 = a - b;
  d2 = 2 * M_PI - fabs(d1);
  if (d1 > 0) d2 *= -1.0;
  if (fabs(d1) < fabs(d2))
    return d1;
  else
    return d2;
}
void PurePersuit::pathCallback(const nav_msgs::Path &currentRefPath) {
  ref_path = currentRefPath;
  pointNum = ref_path.poses.size();
}
void PurePersuit::getRefPath(const nav_msgs::Path &currentRefPath) {
  ref_path = currentRefPath;
  pointNum = ref_path.poses.size();
}
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "pure_persuit");
    ros::NodeHandle nh;
    PurePersuit PurePersuit_(nh);
    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}