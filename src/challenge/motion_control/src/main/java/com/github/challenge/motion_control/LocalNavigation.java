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

import java.awt.Color;

import java.util.ArrayList;
import java.util.List;

import org.ros.node.Node;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.AbstractNodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import gui_msgs.GUILineMsg;
import gui_msgs.GUIPointMsg;
import rss_msgs.BumpMsg;
import rss_msgs.SonarMsg;
import rss_msgs.MotionMsg;

import com.github.rosjava.challenge.gui.SonarGUIPanel;


public class LocalNavigation extends AbstractNodeMain {

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
  protected Subscriber<rss_msgs.OdometryMsg> odoSub;
  protected Publisher<GUIPointMsg> guiPointPub;
  protected Publisher<std_msgs.String> statePub;
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
    int shape = SonarGUIPanel.O_POINT;
    double[] p = getCoordsOffset(SONAR_REL_X[sonar], SONAR_REL_Y[sonar]+range);
    GUIPointMsg pointMsg = guiPointPub.newMessage();
    pointMsg.setX(p[0]);
    pointMsg.setY(p[1]);
    pointMsg.setShape(shape);
    gui_msgs.ColorMsg set_color = pointMsg.getColor();
    set_color.setR(color.getRed());
    set_color.setG(color.getGreen());
    set_color.setB(color.getBlue());
    pointMsg.setColor(set_color);
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
  public void onStart(ConnectedNode node) {
    sonarFrontSub = node.newSubscriber("rss/Sonars/Front", SonarMsg._TYPE);
    sonarBackSub = node.newSubscriber("rss/Sonars/Back", SonarMsg._TYPE);
    bumpSub = node.newSubscriber("rss/BumpSensors", BumpMsg._TYPE);

    statePub = node.newPublisher("/rss/state", std_msgs.String._TYPE);
    motorPub = node.newPublisher("command/Motors", MotionMsg._TYPE);
    guiPointPub = node.newPublisher("gui/Point", GUIPointMsg._TYPE);

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

    odoSub = node.newSubscriber("/rss/odometry", rss_msgs.OdometryMsg._TYPE);
    odoSub
    .addMessageListener(new MessageListener<rss_msgs.OdometryMsg>() {
      @Override
      public void onNewMessage(
          rss_msgs.OdometryMsg message) {
        // Reset robot position and orientation.
        robot.reset(message.getX(), message.getY(), message.getTheta());
        stateHandler.dummyCommand();

        // Let state handler handle state changes due to changes in position.
        stateHandler.handleOdometryUpdate();
      }
    });
  }

  @Override 
  public GraphName getDefaultNodeName() {
    return GraphName.of("rss/motion_control");
  }
}

