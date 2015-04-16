#include "navigation.h"

using namespace rss_msgs;

namespace navigation {

Navigation::Navigation() {
  // TODO fill
  /*
  Navigation navigation;
  ros::Publisher guipoly_pub = n.advertise<gui_msgs::GUIRolyMsg>("gui/Poly", 1000);
  ros::Publisher guierase_pub = n.advertise<gui_msgs::GUIEraseMsg>("gui/Erase", 1000);
  ros::Publisher guipoint_pub = n.advertise<gui_msgs::GUIPointMsg>("gui/Point", 1000);
  // TODO: Change this to the motion commander
  ros::Publisher motor_pub = n.advertise<gui_msgs::GUIRectMsg>("command/Motors", 1000);
// %EndTag(PUBLISHER)%

  std::string mapfile_location;
  assert(n.getParam("~/mapFileName", mapfile_location));

  std::unique_ptr<EnvironmentMap> env(std::make_unique<EnvironmentMap>(mapfile_location));
  std::unique_ptr<Grid> world(std::make_unique<Grid>(&env));

      odoSub = node.newSubscriber("/rss/odometry", OdometryMsg._TYPE);
	    odoSub
	    .addMessageListener(new MessageListener<rss_msgs.OdometryMsg>() {
	      @Override
	      public void onNewMessage(
	          rss_msgs.OdometryMsg message) {
          robot.reset(message.getX(), message.getY(), message.getTheta());
	        nav.handleOdometryMsg(message);
	      }
	    });
      // pass parameter polygonObstacle
      nav.updatePath(ppath);
    }
    //testConvexHull();
  */
}

void Navigation::updateLocation(RobotLocation loc) {
  // TODO
}

void Navigation::moveRobotTo(RobotLocation target) {
  // TODO
}

} // namespace navigation
