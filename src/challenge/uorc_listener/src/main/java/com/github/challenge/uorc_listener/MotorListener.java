package org.github.challenge.uorc_listener;

import org.ros.message.MessageListener;

import MotorControlSolution.RobotVelocityController;
import rss_msgs.MotionMsg;


public class MotorListener implements MessageListener<MotionMsg> {

  private RobotVelocityController controller;

  public MotorListener(RobotVelocityController rvc){
    controller = rvc;
  }
    
  @Override
	public void onNewMessage(MotionMsg msg){

    double trans = msg.getTranslationalVelocity();
    // Convert to travel distance
    double rot = msg.getRotationalVelocity()*(0.433/2.0);
    double left = trans;
    double right = trans;
    
    left -= rot;
    right += rot;
    
    left *= 3.5 / 0.2185; // Divide by c_conversion
    // (read team documentation in google drive)
    right *= 3.5 / 0.2185;


    controller.setDesiredAngularVelocity(left, right);
	
  }   
}
