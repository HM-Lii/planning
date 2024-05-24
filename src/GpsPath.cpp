#include <GpsPath.h>
using namespace std;
using namespace geometry_msgs;
using namespace GeographicLib;

int main(int argc, char** argv) {
  ros::init(argc, argv, "recPath");
  ros::NodeHandle nh;
  std::shared_ptr<GpsPath> gp(new GpsPath(nh));

  ros::Rate rate(1);

  while (ros::ok()) {
    ros::spinOnce();  // 处理ROS消息回调
    switch (gp->GetCmdType()) {
      case GpsPath::CmdType::STOP:
        break;
      case GpsPath::CmdType::GO:
        gp->PublishGoPath();
        break;
      case GpsPath::CmdType::WORK:
        gp->PublishWorkPath();
        break;
      default:
        break;
    }
    rate.sleep();
  }
  return 0;
}