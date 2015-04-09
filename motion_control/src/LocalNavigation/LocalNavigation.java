package LocalNavigation;

import java.awt.Color;

import java.util.ArrayList;
import java.util.List;

import org.ros.message.rss_msgs.BumpMsg;
import org.ros.message.rss_msgs.SonarMsg;
import org.ros.node.Node;

import org.ros.message.MessageListener;
import org.ros.message.all_msgs.GUILineMsg;
import org.ros.message.all_msgs.GUIPointMsg;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;


public class LocalNavigation implements NodeMain {

  // Sonar positions/dimensions of robot.
  public static final double BASE_WIDTH_IN_M = 0.433;
  public static final double[] SONAR_REL_X = {-0.28, 0.025};
  public static final double[] SONAR_REL_Y = {BASE_WIDTH_IN_M/2, BASE_WIDTH_IN_M/2};

  protected double[] responses = new double[2];
  protected double alpha = .7;
  protected double objectThreshold = 1.3;

  // Messaging related variables.
  protected Subscriber<SonarMsg> sonarFrontSub, sonarBackSub;
  protected Subscriber<BumpMsg> bumpSub;
  protected Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
  protected Publisher<GUIPointMsg> guiPointPub;
  protected Publisher<org.ros.message.std_msgs.String> statePub;
  protected Publisher<MotionMsg> motorPub;

  // Keeping state/status across classes.
  protected StateHandler stateHandler;
  // Stores robot position/orientation. StateHandler has a pointer to this instance
  // of the Robot class (any changes here will reflect in StateHandler).
  protected Robot robot;


  public static double[] getCoordsOffset(Robot robot,
                                         double xRange, double yRange){
    double[] p = new double[2];
    p[0] = robot.x + Math.cos(robot.theta)*xRange +
           Math.cos(robot.theta+Math.PI/2.0)*yRange;
    p[1] = robot.y + Math.sin(robot.theta)*xRange +
           Math.sin(robot.theta+Math.PI/2.0)*yRange;
    return p;
  }
  public double[] getCoordsOffset(double xRange, double yRange){
    return getCoordsOffset(robot, xRange, yRange);
  }

  protected void plotSonarReading(double range, int sonar, Color color) {
    int shape = SonarGUI.O_POINT;
    double[] p = getCoordsOffset(SONAR_REL_X[sonar], SONAR_REL_Y[sonar]+range);
    GUIPointMsg pointMsg = new GUIPointMsg();
    pointMsg.x = p[0];
    pointMsg.y = p[1];
    pointMsg.shape = shape;
    pointMsg.color.r = color.getRed();
    pointMsg.color.g = color.getGreen();
    pointMsg.color.b = color.getBlue();
    guiPointPub.publish(pointMsg);
  }

  public boolean isObstacle(double range, int sonar) {
      double iir = (1.0-alpha)*responses[sonar]+alpha*range;
      responses[sonar]=iir;
      return (iir<objectThreshold);
  }

  protected void plotSonarMessage(SonarReading reading) {
    Color color = new Color(0, 0, 0);
    if (reading.isObstacle) {
      color = new Color(0, 255, 0);
    }
    plotSonarReading(reading.range, reading.sonar, color);
  }

  @Override
  public void onStart(Node node) {
    sonarFrontSub = node.newSubscriber("rss/Sonars/Front", "rss_msgs/SonarMsg");
    sonarBackSub = node.newSubscriber("rss/Sonars/Back", "rss_msgs/SonarMsg");
    bumpSub = node.newSubscriber("rss/BumpSensors", "rss_msgs/BumpMsg");

    statePub = node.newPublisher("/rss/state", "std_msgs/String");
    motorPub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
    guiPointPub = node.newPublisher("gui/Point", "all_msgs/GUIPointMsg");

    robot = new Robot();
    stateHandler = new StateHandler(State.SENSING_WALL, node, robot);

    /*
    sonarFrontSub.addMessageListener(new MessageListener<SonarMsg>() {
      @Override
      public void onNewMessage(SonarMsg msg) {
        if (robot.isReady) {
          int sonar = msg.isFront ? 1 : 0;
          SonarReading reading = new SonarReading(sonar, msg.range,
                                                  isObstacle(msg.range, sonar));
          plotSonarMessage(reading);
          stateHandler.handleSonarUpdate(reading);
        }
      }
    });

    sonarBackSub.addMessageListener(new MessageListener<SonarMsg>() {
      @Override
      public void onNewMessage(SonarMsg msg) {
        if (robot.isReady) {
          int sonar = msg.isFront ? 1 : 0;
          SonarReading reading = new SonarReading(sonar, msg.range,
                                                  isObstacle(msg.range, sonar));
          plotSonarMessage(reading);
          stateHandler.handleSonarUpdate(reading);
        }
      }
    });

    bumpSub.addMessageListener(new MessageListener<BumpMsg>() {
      @Override
      public void onNewMessage(BumpMsg message) {
        if (robot.isReady) {
          // Let state handler handle state changes due to bumps.
          stateHandler.handleBumpMsg(message);
        }
      }
    });
    */

    odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
    odoSub
    .addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
      @Override
      public void onNewMessage(
          org.ros.message.rss_msgs.OdometryMsg message) {
        // Reset robot position and orientation.
        robot.reset(message.x, message.y, message.theta);
        stateHandler.dummyCommand();

        // Let state handler handle state changes due to changes in position.
        stateHandler.handleOdometryUpdate();
      }
    });
  }

  @Override 
  public void onShutdown(Node node) {
    if (node != null) {
      node.shutdown();
    }
    System.out.println("not implemented yet?");
  }

  @Override 
  public void onShutdownComplete(Node node) {
    System.out.println("not implemented yet");
  }


  @Override 
  public GraphName getDefaultNodeName() {
    System.out.println("not implemented yet?");
    return new GraphName("rss/localnavigation");
  }
}

