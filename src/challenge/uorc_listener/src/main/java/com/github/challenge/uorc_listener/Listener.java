package org.github.challenge.uorc_listener;

import orc.Orc;

import org.ros.node.NodeConfiguration;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.topic.Subscriber;
import org.apache.commons.logging.Log;
import org.ros.internal.node.topic.PublisherIdentifier;
import org.ros.internal.node.topic.DefaultSubscriber;
import rss_msgs.ArmMsg;
import rss_msgs.MotionMsg;

import MotorControlSolution.RobotBase;
import MotorControlSolution.RobotVelocityController;
import MotorControlSolution.RobotVelocityControllerBalanced;
import java.util.ArrayList;

public class Listener extends AbstractNodeMain {
	
  Orc orc;
  ServoListener sl;
  private Publisher<ArmMsg> armPub;
  private Subscriber<MotionMsg> motorSub;
  private Subscriber<ArmMsg> armSub;

  @Override public void onStart(ConnectedNode node) {

    final Log log = node.getLog();
        
    // TODO Auto-generated method stub
    //Motor Control
    try{
        
      System.out.println("in main");
      orc = Orc.makeOrc(); //this orc is used only by the servo controller, the robotbase makes its own
      RobotBase robot = new RobotBase();
      
      System.out.println("robot base made");
      
      RobotVelocityController robotVelocityController = null;
      robotVelocityController = new RobotVelocityControllerBalanced();
      System.out.println("robot velocity controller created");
      robot.setRobotVelocityController(robotVelocityController);
      robotVelocityController.setGain(1);
      for(int i = 0; i < 2; i++){
        robotVelocityController.getWheelVelocityController(i).setGain(6);
      }
      robot.enableMotors(true);
      
      motorSub = node.newSubscriber("command/Motors", MotionMsg._TYPE);
      motorSub.addMessageListener(new MotorListener(robotVelocityController));
      log.info("motor Subscriber created");

      armPub = node.newPublisher("rss/ArmStatus", ArmMsg._TYPE);

      sl = new ServoListener(orc, armPub, true);//to use safe servos set to true
      //this requires modification of the ServoListener class to have the correct upper and lower bounds	    
      armSub = node.newSubscriber("command/Arm", ArmMsg._TYPE);
      armSub.addMessageListener(sl);
      log.info("arm subscriber created");

      /*
      subMonitorThreadBase = new SubMonitorThread();
      subMonitorThread = new Thread(this.subMonitorThreadBase);
      subMonitorThread.start();
      */
      
    } catch (Exception e) {
      e.printStackTrace();
    }

  }

    /*
    private SubMonitorThread subMonitorThreadBase;
    private Thread subMonitorThread; 

    private class SubMonitorThread implements Runnable{
	@Override
	    public void run() {
	    while (true){
		synchronized(armSub)  {
		    java.util.Collection<PublisherIdentifier> publishers = new ArrayList<PublisherIdentifier>();
		    DefaultSubscriber<?> sub = (DefaultSubscriber<?>)armSub;
		    sub.updatePublishers(publishers);
		    System.out.println(publishers);
		}
		try {
		    Thread.sleep(50);
		} catch (InterruptedException e) {
		    e.printStackTrace();
		}
	    }
	}
    }
    */

  @Override public GraphName getDefaultNodeName() {
    return GraphName.of("rss/uorc_listener");
  }

}
