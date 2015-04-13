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

package com.github.rosjava.challenge.navigation;

import java.awt.geom.*;
import java.util.*;

/***
 * <p> C-Space Transformation Engine. </p>
 *
 * @author Ali-Amir Aldan
 **/
public class CSTools {
  /***
   * <p>Method that computes C-Space transformation of the real obstacles
   * given as List<PolygonObstacle> rObst. Returns a list of obstacles in
   * C-Space coordinates.</p>
   *
   * @param rObst       - list of obstacles
   *
   * @return list of obstacles in c-space coordinates
   **/
  public static List<CSObstacle> getCSTransform(
      List<PolygonObstacle> rObst) {
    ArrayList<CSObstacle> transform = new ArrayList<CSObstacle>();
    for (PolygonObstacle obst : rObst) {
      transform.add(new CSObstacle(obst));
    }
    return transform;
  }

  /***
   * <p>Returns a polygon representing robot in real space coordinates,
   * after a rotation by theta.</p>
   *
   * @return polygon representing the robot
   **/
  public static PolygonObstacle getRobotRepresentation(double theta) {
    PolygonObstacle robot = new PolygonObstacle();
    double pForward = 0.21, pBackward = -0.35,
           pLeft = -0.25, pRight = 0.25;
    // (pLeft, pForward) rotated by theta.
    robot.addVertex(new Point2D.Double(
        pLeft*Math.cos(theta)-pForward*Math.sin(theta),
        pLeft*Math.sin(theta)+pForward*Math.cos(theta)));
    // (pRight, pForward) rotated by theta.
    robot.addVertex(new Point2D.Double(
        pRight*Math.cos(theta)-pForward*Math.sin(theta),
        pRight*Math.sin(theta)+pForward*Math.cos(theta)));
    // (pLeft, pBackward) rotated by theta.
    robot.addVertex(new Point2D.Double(
        pLeft*Math.cos(theta)-pBackward*Math.sin(theta),
        pLeft*Math.sin(theta)+pBackward*Math.cos(theta)));
    // (pRight, pBackward) rotated by theta.
    robot.addVertex(new Point2D.Double(
        pRight*Math.cos(theta)-pBackward*Math.sin(theta),
        pRight*Math.sin(theta)+pBackward*Math.cos(theta)));
    return robot;
  }

  /***
   * <p>Transforms a given obstacle from real world coordinates to C-Space
   * coordinates.</p>
   *
   * @param obstacle    - polygon representing the obstacle
   * @param theta       - robot's orientation
   *
   * @return  obstacle in c-space coordinates
   **/
  public static PolygonObstacle getCSTransform(
      PolygonObstacle obstacle, double theta) {
    PolygonObstacle robot = getRobotRepresentation(theta); 
    PolygonObstacle mink = minkowskiSum(obstacle, reflectAboutOrigin(robot));
    List<Point2D.Double> minkP = mink.getVertices();
    PolygonObstacle transform = new PolygonObstacle();
    for (Point2D point : minkP) {
      transform.addVertex(new Point2D.Double(point.getX(), point.getY()));
    }
    transform.close();
    return transform;
  }

  /***
   * <p>Reflects a given polygon about a origin.</p>
   *
   * @param object  - the polygon to be inverted
   *
   * @return  inverted polygon
   **/
  public static PolygonObstacle reflectAboutOrigin(PolygonObstacle object) {
    List<Point2D.Double> objP = object.getVertices();
    PolygonObstacle reflection = new PolygonObstacle();
    for (Point2D point : objP) {
      reflection.addVertex(new Point2D.Double(-point.getX(),
                                              -point.getY()));
    }
    reflection.close();
    return reflection;
  }

  /***
   * <p>Returns convex hull of a Minkowski sum of two polygons.</p>
   *
   * @param polyA   - first polygon
   * @param polyB   - second polygon
   *
   * @return  convex hull of minkowski sum of the two
   **/
  public static PolygonObstacle minkowskiSum(
      PolygonObstacle polyA, PolygonObstacle polyB) {
    List<Point2D.Double> ap = polyA.getVertices();
    List<Point2D.Double> bp = polyB.getVertices();
    List<Point2D.Double> sumAB = new LinkedList<Point2D.Double>();
    for (Point2D pointA : ap) {
      for (Point2D pointB : bp) {
        sumAB.add(new Point2D.Double(pointA.getX() + pointB.getX(),
                                     pointA.getY() + pointB.getY()));
      }
    }
    return GeomUtils.convexHull(sumAB);
  }
}
