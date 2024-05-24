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
  path_sub_ = nh_.subscribe("/point_gen/ref_path", 20, &PurePersuit::pathCallback, this);
  cout << "listening to odometry!!" << endl;
  nh_.param("preview_distance", preview_distance_, 2.0);
  nh_.param("unit_length", unit_length_, 0.1);
  nh_.param("velocity", vel_, 0.3);
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
  if (enableObstacleAvoidance && if_obs) {
    cout << "warning: obstacle !!" << endl;
    publishVelocityMessage(0, 0);
    return;
  }
  auto currentPoint = currentWaypoint->pose.pose;
  geometry_msgs::Pose nearestPoint;
  geometry_msgs::PoseStamped previewPoint;
  findNearestAndPreviewPoints(currentPoint, nearestPoint, previewPoint);
  cout << "currentPoint: " << currentPoint.position.x << ","
       << currentPoint.position.y << endl;
  double heading_goal =
      atan2((previewPoint.pose.position.y - currentPoint.position.y),
            (previewPoint.pose.position.x - currentPoint.position.x));
  double current_heading;
  if (enable_gps) {
    heading_goal = -(heading_goal - (M_PI / 2));
    current_heading = current_heading_;
  } else {
    current_heading = tf::getYaw(currentPoint.orientation);
  }
  float delta_angle = calculateDeltaAngle(heading_goal,current_heading);
  // 当前点和目标点的距离Id
  cout << "heading_goal: " << heading_goal;
  cout << "; current_heading: " << current_heading;
  cout << "; delta_angle: " << delta_angle << endl;
  float dis_cur_pre =
      sqrt(pow(previewPoint.pose.position.y - currentPoint.position.y, 2) +
           pow(previewPoint.pose.position.x - currentPoint.position.x, 2));
  // 发布运动指令:线速度和角速度
  if (dis_cur_pre > SLOW_DOWN_DISTANCE) {
    if (fabs(delta_angle) < ANGLE_THRESHOLD) {// 如果分母为0或接近于0，则无法计算变量R的值
      publishVelocityMessage(vel_, 0);// 此时直线行驶
    } else {
      float R = dis_cur_pre / (2 * sin(delta_angle));
      publishVelocityMessage(vel_, vel_ / R);
    }
    if_reach = false;
  } else if (dis_cur_pre > STOP_DISTANCE) {
    publishVelocityMessage(vel_ / 2, 0);// 减速
    cout << "close to the end, slowing down!!" << endl;
    if_reach = false;
  } else {
    publishVelocityMessage(0, 0);// 停止
    if_reach = true;
    cout << "reach the goal!!" << endl;
  }
}
void PurePersuit::headingCallback(const geometry_msgs::Pose2D::ConstPtr& currentHeading) {
  current_heading_ = currentHeading->theta;
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
  double nearestDis = 1000000;
  for (int i = 0; i < pointNum; i++) {
    double dx = ref_path.poses[i].pose.position.x - currentPoint.position.x;
    double dy = ref_path.poses[i].pose.position.y - currentPoint.position.y;
    if (dx * dx + dy * dy < nearestDis) {
      nearestDis = dx * dx + dy * dy;
      nearestPoint = ref_path.poses[i].pose;
      nearestIndex = i;
    }
  }
  int preview_index;
  int preview_num = preview_distance_ / unit_length_;
  if(preview_num >= pointNum - nearestIndex)
    preview_num = pointNum - nearestIndex-1;
  preview_index = nearestIndex+preview_num;
  previewPoint.header.frame_id = "map";
  previewPoint.header.stamp = ros::Time::now();
  previewPoint.pose = ref_path.poses[preview_index].pose;
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