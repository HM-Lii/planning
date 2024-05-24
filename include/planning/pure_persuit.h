#ifndef PURE_PERSUIT_H
#define PURE_PERSUIT_H

#include <ros/ros.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <serial/serial.h>

class PurePersuit {
public:
    PurePersuit(ros::NodeHandle& nh_);
    void poseCallback(const nav_msgs::Odometry::ConstPtr& currentWaypoint);
    void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& currentFix);
    void headingCallback(const geometry_msgs::Pose2D::ConstPtr& currentHeading);
    void pathCallback(const nav_msgs::Path& currentRefPath);
    void getRefPath(const nav_msgs::Path& currentRefPath);
    void obs(){if_obs=true;}
    void no_obs(){if_obs=false;}
    void reach(){if_reach=true;}
private:
    std::vector<geometry_msgs::Pose> calculateArcPoints(const geometry_msgs::Pose &currentPoint,
                                                        const geometry_msgs::Pose &previewPoint,
                                                        float linear_velocity,
                                                        float angular_velocity,
                                                        float R, 
                                                        float current_heading,
                                                        float dis_cur_pre);
    void publishVelocityMessage(float linear, float angular) ;
    void findNearestAndPreviewPoints(const geometry_msgs::Pose &currentPoint,
                                     geometry_msgs::Pose &nearestPoint,
                                     geometry_msgs::PoseStamped &previewPoint);
    float calculateDeltaAngle(float a, float b);

    ros::Publisher vel_pub_, preview_pose_pub_;
    ros::Subscriber pose_sub_,path_sub_,heading_sub_,fix_sub_;
    nav_msgs::Path ref_path;
    int pointNum = 1,check_count = 0;
    double vel_;
    double preview_distance_,unit_length_;
    double current_heading_;
    bool if_reach = false;
    bool if_obs=false;
    bool enable_gps=true;
    bool enableObstacleAvoidance = false;
    const double NEAREST_DIS_INIT = 1000000;
    const double SLOW_DOWN_DISTANCE = 0.6;
    const double STOP_DISTANCE = 0.1;
    const double ANGLE_THRESHOLD = 1e-3;
};

#endif