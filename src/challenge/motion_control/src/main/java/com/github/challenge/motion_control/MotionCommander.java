/*
 * Copyright (C) 2014 Ali-Amir Aldan.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.github.rosjava.challenge.motion_control;

import rss_msgs.MotionMsg;
import org.ros.node.topic.Publisher;
import org.ros.node.ConnectedNode;

public class MotionCommander {

  /**
   * <p>The maximum pwm command magnitude.</p>
   **/
  protected static final double MAX_PWM = 255;

  /**
   * <p>Student Code: unloaded maximum wheel angular velocity in rad/s. 
   * This should be a protected static final double called MAX_ANGULAR_VELOCITY.</p>
   **/
  protected static final double MAX_ANGULAR_VELOCITY = 8.17466;

  /**
   * <p>Student Code: radius of the wheel on the motor (in meters). 
   * This should be a protected static final double called WHEEL_RADIUS_IN_M.</p>
   **/
  protected static final double WHEEL_RADIUS_IN_M = 0.0635;

  /**
   * <p>Student Code: encoder resolution.</p>
   * This should be a protected static final double called ENCODER_RESOLUTION.</p>
   * Hint: ENCODER_RESOLUTION units: ticks/revolution (WITHOUT gear ratio term)
   **/
  protected static final double ENCODER_RESOLUTION = 1997.700;

  /**
   * <p>Student Code: motor revolutions per wheel revolution.
   * This should be a protected static final double called GEAR_RATIO.</p>
   **/
  protected static final double GEAR_RATIO = 65.5;

  /**
   * <p>Student Code: encoder ticks per motor revolution.</p>
   * This should be a protected static final double called TICKS_PER_REVOLUTION.</p>
   * Hint: TICKS_PER_REVOLUTION units: ticks/revolution (WITH gear ratio term)
   **/
  protected static final double TICKS_PER_REVOLUTION = 130849.333;

  /**
   * <p>Student Code: width of the base of the robot in meters.
   * This should be a protected static final double called GEAR_RATIO.</p>
   **/
  protected static final double BASE_WIDTH_IN_M = 0.433;

  protected Robot robot;

  protected Publisher<MotionMsg> motorPub;

  protected int currentCommand = 0;
  protected MotionCommand[] commands = {};

  public MotionCommander(ConnectedNode node, Robot robotPtr) {
    motorPub = node.newPublisher("command/Motors", MotionMsg._TYPE);
    robot = robotPtr;
  }

  public void command(MotionCommand newCommand) {
    MotionCommand[] newCommands = {newCommand};
    command(newCommands);
  }

  public void command(MotionCommand[] newCommands) {
    commands = newCommands;
    if (commands.length > 0) {
      commands[0].init(robot.x, robot.y, robot.theta);
    }
    currentCommand = 0;
    step();
  }

  public void step() {
    if (currentCommand < commands.length) {
      double transVel = commands[currentCommand].getTranslationalVelocity(
                                                robot.x, robot.y, robot.theta);
      transVel = convertTranslationalToAngularVelocity(transVel);
      // TODO: Scale by conversion factor

      double rotVel = commands[currentCommand].getRotationalVelocity(
                                                robot.x, robot.y, robot.theta);
      rotVel = convertBaseAngVelToWheelAngVel(rotVel);
      // TODO: Scale by conversion factor

      MotionMsg motionMsg = motorPub.newMessage();
      motionMsg.setTranslationalVelocity(transVel);
      motionMsg.setRotationalVelocity(rotVel);
      motorPub.publish(motionMsg);
    }
    isDone();
  }

  public boolean isDone() {
    while (currentCommand < commands.length) {
      if (!commands[currentCommand].isDone(robot.x, robot.y, robot.theta)) {
        break;
      }
      ++currentCommand;
      if (currentCommand < commands.length) {
        commands[currentCommand].init(robot.x, robot.y, robot.theta);
      }
    }
    return currentCommand == commands.length;
  }

  /**
   * <p>Computes the angular velocity in rad/s given translational velocity.</p>
   *
   * @param translationalVelocity translational velocity in m/s
   *
   * @return the angular velocity in rad/s, positive means wheel turned CCW
   * when observing wheel from the outside (non-motor side)
   **/
  public static double convertTranslationalToAngularVelocity(
      double translationalVelocity) {
    return translationalVelocity / WHEEL_RADIUS_IN_M;
  }

  /**
   * <p>Converts rotational velocity of the base to the corresponding rotational
   * velocity of a wheel.</p>
   *
   * @param baseAngVelocity angular velocity of the base in rad/s
   *
   * @return wheel rotational velocity in rad/s
   **/
  public static double convertBaseAngVelToWheelAngVel(double baseAngVelocity) {
    return baseAngVelocity*(BASE_WIDTH_IN_M/2.0) / WHEEL_RADIUS_IN_M;
  }

  public static double getCommandW(double w) {
    return w * LocalNavigation.BASE_WIDTH_IN_M/2.0;
  }
}
