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
import java.awt.geom.Point2D;

import gui_msgs.GUISegmentMsg;
import gui_msgs.GUILineMsg;
import org.ros.node.topic.Publisher;
import org.ros.node.ConnectedNode;

public class WallModel {
  protected int sum = 0;
  protected double xsum = 0;
  protected double ysum = 0;
  protected double x2sum = 0;
  protected double y2sum = 0;
  protected double xysum = 0;

  protected double aNorm = 0;
  protected double bNorm = 0;
  protected double cNorm = 0;
  
  protected int disCount = 0;
  protected double disSum = 0.0;

  protected double minD;
  protected double maxD;
  protected boolean isStabilized = false;

  protected Publisher<GUILineMsg> guiLinePub;
  protected Publisher<GUISegmentMsg> guiSegmentPub;

  WallModel(ConnectedNode node) {
    guiLinePub = node.newPublisher("gui/Line", GUILineMsg._TYPE);
    guiSegmentPub = node.newPublisher("gui/Segment", GUISegmentMsg._TYPE);
  }

  public boolean isWallStable() {
    return isStabilized;
  }

  public boolean isPartOfWall(double range, double x, double y) {
    double currentObstacleDistanceInM = Math.abs(x*aNorm + y*bNorm + cNorm);

    if (isStabilized && currentObstacleDistanceInM < 0.05) {
      double xC = 0.0;
      double yC = -cNorm/bNorm;
      double d = (x - xC)*bNorm - (y - yC)*aNorm;
      minD = Math.min(minD, d);
      maxD = Math.max(maxD, d);
    }

    return currentObstacleDistanceInM < 0.05; // 5 cm
  }

  public double getDistanceToWall() {
    if (disCount == 0) {
      return 1.0;
    }
    return disSum / disCount;
  }

  public void updateDistance(double x, double y) {
    double newD = Math.abs(aNorm*x+bNorm*y+cNorm);
    disSum += newD;
    ++disCount;
  }

  public void drawWall() {
    GUISegmentMsg msg = guiSegmentPub.newMessage();
    WallSegment w = getWallSegment();
    msg.setStartX(w.getStartPoint().getX());
    msg.setStartY(w.getStartPoint().getY());
    msg.setEndX(w.getEndPoint().getX());
    msg.setEndY(w.getEndPoint().getY());
    Color color = SonarGUIPanel.makeRandomColor();
    gui_msgs.ColorMsg set_color = msg.getColor();
    set_color.setR(color.getRed());
    set_color.setG(color.getGreen());
    set_color.setB(color.getBlue());
    msg.setColor(set_color);
    guiSegmentPub.publish(msg);
  }

  public WallSegment getWallSegment() {
    double xC = 0.0;
    double yC = -cNorm/bNorm;
    Point2D start = new Point2D.Double(xC + bNorm*minD, yC - aNorm*minD);
    Point2D end = new Point2D.Double(xC + bNorm*maxD, yC - aNorm*maxD);
    return new WallSegment(start, end, aNorm, bNorm, cNorm);
  }

  public void resetModel() {
    sum = 0;
    xsum = 0;
    ysum = 0;
    x2sum = 0;
    y2sum = 0;
    xysum = 0;
    aNorm = 0;
    bNorm = 0;
    cNorm = 0;
    minD = 1e18;
    maxD = -1e18;
  }

  public void addObservation(double x, double y) {
    xsum += x;
    x2sum += x*x;
    ysum += y;
    y2sum += y*y;
    xysum += x*y;
    sum += 1;

    double d = (x2sum*sum - xsum*xsum);

    if (Math.abs(d) < 1e-9) {
      return;
    }
  
    double a = (xysum*sum - xsum*ysum)/d;
    double b = (x2sum*ysum  - xysum*xsum)/d;
    double normalizer = Math.sqrt(a*a + 1.0);
    double aOld = aNorm;
    double bOld = bNorm;
    double cOld = cNorm;
    aNorm = a/normalizer;
    bNorm = -1.0/normalizer;
    cNorm = b/normalizer;
/*
    if (Math.abs(Math.abs(aNorm*aOld+bNorm*bOld)-1.0) < 0.1 &&
        Math.abs(Math.abs(cNorm)-Math.abs(cOld)) < 0.2*Math.abs(cOld) - 1e-5) {
      isStabilized = true;
    }
    */
    if (sum == 80) {
      isStabilized = true;
    }

    GUILineMsg lineMsg = guiLinePub.newMessage();
    lineMsg.setLineA(aNorm);
    lineMsg.setLineB(bNorm);
    lineMsg.setLineC(cNorm);
    gui_msgs.ColorMsg set_color = lineMsg.getColor();
    set_color.setR(0);
    set_color.setG(0);
    set_color.setB(255);
    lineMsg.setColor(set_color);
    guiLinePub.publish(lineMsg);
  }

}
