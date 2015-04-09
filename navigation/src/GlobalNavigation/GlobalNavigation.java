package GlobalNavigation;

import java.awt.Color;
import java.awt.geom.*;
import java.util.ArrayList;
import java.util.List;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.namespace.GraphName;
import org.ros.node.*;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.node.parameter.ParameterTree;
import org.ros.message.all_msgs.GUILineMsg;
import org.ros.message.all_msgs.GUIPointMsg;
import org.ros.message.all_msgs.GUISegmentMsg;
import org.ros.message.all_msgs.GUIEraseMsg;
import org.ros.message.rss_msgs.BumpMsg;
import org.ros.message.rss_msgs.SonarMsg;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.message.all_msgs.GUIPolyMsg;
import org.ros.message.all_msgs.GUIRectMsg;

import LocalNavigation.StateHandler;

public class GlobalNavigation implements NodeMain {
  protected Publisher<GUIRectMsg> guiRectPub;
  protected Publisher<GUIPolyMsg> guiPolyPub;
  protected Publisher<GUIEraseMsg> guiErasePub;
  protected Publisher<GUIPointMsg> guiPointPub;
  protected Publisher<MotionMsg> motorPub;
  protected Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;

  protected Publisher<GUISegmentMsg> guiSegmentPub;
  

  protected PolygonMap polygonMap;
  protected MotionPlanner motionPlanner;
  protected List<Point2D.Double> path;
  protected LocalNavigation.Robot robot;
  protected WaypointNav nav; 
  
  protected double resolution = 0.02;
  
  @Override
  public void onStart(Node node) {
    
    guiRectPub = node.newPublisher("gui/Rect", "all_msgs/GUIRectMsg");
    guiPolyPub = node.newPublisher("gui/Poly", "all_msgs/GUIPolyMsg");
    guiErasePub = node.newPublisher("gui/Erase", "all_msgs/GUIEraseMsg");
    guiPointPub = node.newPublisher("gui/Point", "all_msgs/GUIPointMsg");

    motorPub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
    robot = new LocalNavigation.Robot();
    nav = new WaypointNav(Robot_State.READY, node, robot);

    guiSegmentPub = node.newPublisher("gui/Segment", "all_msgs/GUISegmentMsg");
    
	  ParameterTree paramTree = node.newParameterTree();    
	  String mapFileName = paramTree.getString(node.resolveName("~/mapFileName"));

    try {
      Thread.sleep(10000); //Wait 10 seconds to let MapGUI initialize, otherwise it won't listen to messages on time
      polygonMap = new PolygonMap(mapFileName);      
      displayMap(polygonMap);
      
      List<PolygonObstacle> mapObstacles = getMapBoundariesAsPolygons(polygonMap.getWorldRect());
      mapObstacles.addAll(polygonMap.getObstacles());
      List<CSObstacle> csObstacles = CSTools.getCSTransform(mapObstacles);

      RandomNet net = new RandomNet(polygonMap.getWorldRect(), csObstacles, 100);
      displayRandomNet(net);

      List<PolygonObstacle> noThetaObst = new ArrayList<PolygonObstacle>();
      for (CSObstacle obst : csObstacles) {
        noThetaObst.add(obst.get(0));
      }
      for (PolygonObstacle obstacle : noThetaObst) {
        obstacle.color = new Color(255, 0, 0);
        System.out.println(obstacle);
      }
      System.out.println("I'm going to break after");
      publishPolygonObstacle(noThetaObst);
      System.out.println("Starting MotionPlanning");
      motionPlanner = new MotionPlanner(polygonMap.getWorldRect(), resolution, noThetaObst);
      System.out.println("Starting Pathmaking");
      path = motionPlanner.computePath(polygonMap.getRobotStart(), polygonMap.getRobotGoal());
      System.out.println("Finished Pathmaking");

      PolygonObstacle ppath = createPathPolygon(path);
      publishPolygonObstacle(ppath);
      
      odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
	    odoSub
	    .addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
	      @Override
	      public void onNewMessage(
	          org.ros.message.rss_msgs.OdometryMsg message) {
          robot.reset(message.x, message.y, message.theta);
	        nav.handleOdometryMsg(message);
	      }
	    });
      // pass parameter polygonObstacle
      nav.updatePath(ppath);
    }
    catch (Exception e) {
      System.out.println("GlobalNavigation, error loading PolygonMap from file: " + e.toString());
    }
    //testConvexHull();
  }  
  
  private List<PolygonObstacle> getMapBoundariesAsPolygons(Rectangle2D.Double worldRect) {
    List<PolygonObstacle> mapBoundaries = new ArrayList<PolygonObstacle>();
    
    PolygonObstacle bottom = new PolygonObstacle();
    bottom.addVertex(worldRect.getX(), worldRect.getY());
    bottom.addVertex(worldRect.getX() + worldRect.getWidth(), worldRect.getY());
    bottom.close();
    mapBoundaries.add(bottom);
 
    PolygonObstacle top = new PolygonObstacle();
    top.addVertex(worldRect.getX(), worldRect.getY() + worldRect.getHeight());
    top.addVertex(worldRect.getX() + worldRect.getWidth(), worldRect.getY() + worldRect.getHeight());
    top.close();
    mapBoundaries.add(top);
    
    PolygonObstacle left = new PolygonObstacle();
    left.addVertex(worldRect.getX(), worldRect.getY());
    left.addVertex(worldRect.getX(), worldRect.getY() + worldRect.getHeight());
    left.close();
    mapBoundaries.add(left);
 
    PolygonObstacle right = new PolygonObstacle();
    right.addVertex(worldRect.getX() + worldRect.getWidth(), worldRect.getY());
    right.addVertex(worldRect.getX() + worldRect.getWidth(), worldRect.getY() + worldRect.getHeight());
    right.close();
    mapBoundaries.add(right);
    
    return mapBoundaries;
  }
  
  private void publishWorldRectangle(Rectangle2D.Double worldRect) {
    GUIRectMsg rectMsg = new GUIRectMsg();
    rectMsg.c.r = 0;
    rectMsg.c.g = 255;
    rectMsg.c.b = 0;    
    rectMsg.x = (float)worldRect.getX();
    rectMsg.y = (float)worldRect.getY();
    rectMsg.width = (float)worldRect.getWidth();
    rectMsg.height = (float)worldRect.getHeight();
    rectMsg.filled = false ? 1 : 0;
    System.out.println("sending world rectangle message");
    guiRectPub.publish(rectMsg);    
  }
  
  private void publishPolygonObstacle(List<PolygonObstacle> obstacles) {
    for(PolygonObstacle obstacle : obstacles) {
      publishPolygonObstacle(obstacle);
    }
  }
  
  private void publishPolygonObstacle(PolygonObstacle obstacle) {
      GUIPolyMsg polyMsg = new GUIPolyMsg();
           
      List<Point2D.Double> vertices = obstacle.getVertices();
      polyMsg.numVertices = vertices.size();
      polyMsg.x = new float[vertices.size()];
      polyMsg.y = new float[vertices.size()];
      for(int i = 0; i < vertices.size(); i++) {
        polyMsg.x[i] = (float)vertices.get(i).x;
        polyMsg.y[i] = (float)vertices.get(i).y;
      }
      
      if (obstacle.color != null) {
        polyMsg.c.r = obstacle.color.getRed();
        polyMsg.c.g = obstacle.color.getGreen();
        polyMsg.c.b = obstacle.color.getBlue();
      } else {
        polyMsg.c.r = 255;
        polyMsg.c.g = 0;
        polyMsg.c.b = 0;
      }
      
      polyMsg.closed = obstacle.closed ? 1 : 0;
      polyMsg.filled = false ? 1 : 0;
      
      System.out.println("sending polygon obstacle message");
      guiPolyPub.publish(polyMsg);
  }

  private void displayRandomNet(RandomNet net) {
    GUIPointMsg pointMsg = new GUIPointMsg();
    pointMsg.shape = LocalNavigation.SonarGUI.O_POINT;
    pointMsg.color.r = 0;
    pointMsg.color.g = 0;
    pointMsg.color.b = 255;
    for (CSCoord point : net.getPoints()) {
      pointMsg.x = point.x();
      pointMsg.y = point.y();
      guiPointPub.publish(pointMsg);
    }
  }
 
  private void displayMap(PolygonMap polygonMap) {
    publishWorldRectangle(polygonMap.getWorldRect());
    publishPolygonObstacle(polygonMap.getObstacles());	

    GUIPointMsg pointMsg = new GUIPointMsg();
    pointMsg.x = polygonMap.getRobotStart().x;
    pointMsg.y = polygonMap.getRobotStart().y;
    pointMsg.shape = LocalNavigation.SonarGUI.O_POINT;
    pointMsg.color.r = 255;
    pointMsg.color.g = 0;
    pointMsg.color.b = 0;
    guiPointPub.publish(pointMsg);

    GUIPointMsg pointMsg2 = new GUIPointMsg();
    pointMsg2.x = polygonMap.getRobotGoal().x;
    pointMsg2.y = polygonMap.getRobotGoal().y;
    pointMsg2.shape = LocalNavigation.SonarGUI.O_POINT;
    pointMsg2.color.r = 0;
    pointMsg2.color.g = 0;
    pointMsg2.color.b = 255;
    guiPointPub.publish(pointMsg2);

  }

  private PolygonObstacle createPathPolygon(List<Point2D.Double> path){
    System.out.println("///Printing path with " + path.size() +
                       " points.////");
    PolygonObstacle ppath = new PolygonObstacle();
    for (int i = 0; i < path.size(); ++i) {
      ppath.addVertex(path.get(i));
    }
    ppath.color = new Color(0, 255, 255);
   return ppath;
  }

  @Override 
  public void onShutdown(Node node) {
    System.out.println("GlobalNavigation.onShutdown: not implemented yet?");
    if (node != null) {
      node.shutdown();
    }    
  }

  @Override 
  public void onShutdownComplete(Node node) {
    System.out.println("GlobalNavigation.onShutdownComplete: not implemented yet");
  }

  @Override 
  public GraphName getDefaultNodeName() {
    return new GraphName("rss/globalnavigation");
  }
  
  public void handle(BumpMsg arg0) {
    //TODO: implement this. Need this to compile. Look at BumpMsg.java in this directory, line 17
	}
  
  public void handle(OdometryMsg arg0) {
    //TODO: implement this. Need this to compile. Look at OdometryListener.java in this directory, line 18
	}
  
  public static void fillRectMsg(GUIRectMsg rectMsg, Rectangle2D.Double worldRect, Object unknown_type_or_anything, boolean unknown_boolean) {
    //TODO: fill up rectMsg with something. Need this method to compile. Look at PolygonMap.java, line 400
  }
  
  public static void fillPolyMsg(GUIPolyMsg polyMsg, PolygonObstacle obstacle, Color color, boolean unknown_boolean1, boolean unknown_boolean2) {
    //TODO: implement this in some way. Need this method to compile. Look at PolygonMap.java, line 405
  }  
  
  private void testConvexHull() {
    double[] x = {0, 
                  2.9405490840,
                  2.8687990640,
                  -5.1062731580,
                  -1.7541145700,
                  -9.1376159840,
                  8.1576698450,
                  -3.4411852360,
                  3.3110390510,
                  -9.2685174680,
                  -1.1005548710,
                  7.8202537780,
                  1.1420850980,
                  -3.3023341220,
                  -4.1824489540,
                  1.8158106440,
                  -1.0185118370,
                  -1.7196053850,
                  1.8098914660,
                  2.2158257470,
                  -1.3205126820,
                  -7.6630448980,
                  -1.2627365050,
                  -4.2742015680,
                  1.4916968880,
                  -7.2593661990,
                  4.7257136580,
                  -1.2081584300,
                  -3.0343858400,
                  -1.3765507720,
                  -6.9032124010,
                  -6.2725483390,
                  2.2589410430,
                  -1.5947838440,
                  6.2454906590,
                  -8.9730348620,
                  -2.8768047430,
                  5.2062792460,
                  1.9333773520,
                  -7.5928118290,
                  4.0427115780,
                  8.0101004600,
                  -8.5417585870,
                  -7.6940623090,
                  1.2241122440,
                  2.7924382490,
                  1.8980953690,
                  6.2636151410,
                  -5.1987423930,
                  -1.2671168960,
                  9.5822900840};
 
    double[] y = {0,
                  1.1762781120,
                  -8.5004964010,
                  6.1698661940,
                  1.2891518430,
                  -1.8812864580,
                  -5.7245319880,
                  -6.7032259540,
                  5.8427132300,
                  9.2315714230,
                  5.6998229290,
                  -1.1500836750,
                  1.4877525110,
                  4.7567169260,
                  1.1216307170,
                  1.9454300280,
                  3.8923144320,
                  -1.0853726370,
                  -1.2282088880,
                  1.7886977510,
                  4.8982213320,
                  -6.8495875950,
                  3.8799599880,
                  -1.4972802390,
                  1.4824101030,
                  6.7060140210,
                  5.2066716060,
                  -1.8602336320,
                  9.1450399200,
                  1.1727606450,
                  -1.3952093870,
                  -1.1586457240,
                  -7.6644334710,
                  1.1120456740,
                  4.5789172860,
                  8.3800225810,
                  4.2005978660,
                  -2.1964260440,
                  7.2913036330,
                  1.7451301100,
                  -1.1314779310,
                  -2.5636663630,
                  6.8427367500,
                  1.7863646690,
                  -7.0792601050,
                  -2.7353854120,
                  -5.6505672810,
                  9.2608504810,
                  9.3288650470,
                  3.0725271120,
                  1.4885808470};
    
    List<Point2D.Double> points = new ArrayList<Point2D.Double>();
    for(int i = 0; i < x.length; i++) {
      points.add(new Point2D.Double(x[i] % 0.1, y[i] % 0.1));
      
      GUIPointMsg pointMsg = new GUIPointMsg();
      pointMsg.x = x[i] % 0.1;
      pointMsg.y = y[i] % 0.1;
      pointMsg.shape = LocalNavigation.SonarGUI.X_POINT;
      pointMsg.color.r = 0;
      pointMsg.color.g = 0;
      pointMsg.color.b = 255;
      guiPointPub.publish(pointMsg);
    }
    
    PolygonObstacle convexHull = GeomUtils.convexHull(points);
    publishPolygonObstacle(convexHull);
  }
}
