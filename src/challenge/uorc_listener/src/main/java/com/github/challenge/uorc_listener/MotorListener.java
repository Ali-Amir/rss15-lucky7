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
	
    double left = msg.getTranslationalVelocity();
    double right = msg.getTranslationalVelocity();
    
    left -= msg.getRotationalVelocity();
    right += msg.getRotationalVelocity();
    
    left *= 3.5;
    right *= 3.5;
    controller.setDesiredAngularVelocity(left, right);
	
  }   
}
