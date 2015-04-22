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
import java.awt.Color;

public class MotionPlanner {

  public CSGrid grid;

  Point2D.Double currPoint;
  Point2D.Double addPoint;
  Point2D.Double checkPoint;


  MotionPlanner(Rectangle2D.Double worldBounds, int angleDiv, double resolution, List<CSObstacle> obstacles) {
    grid = new CSGrid(worldBounds, angleDiv, resolution);

    for (CSObstacle obstacle: obstacles){
      grid.markObstacle(obstacle);
    }
  }

  List<Point2D.Double> computePath(CSCoord robotPoint, CSCoord goalPoint){
    List<Point2D.Double> path = new ArrayList<Point2D.Double>();
    List<Point2D.Double> shortPath = new ArrayList<Point2D.Double>();
    CSGrid.Cell goalCell = grid.getCell(goalPoint);
    CSGrid.Cell robotCell = grid.getCell(robotPoint);
    
    double maxDist = grid.computeShortestPaths(goalPoint);
   
    if (grid.getCell(robotPoint).toGoalNext==null){
    	System.out.println("No path!!!");
      return path;
    } else {
      if (grid.getCell(robotPoint).toGoalNext!=null){

        CSGrid.Cell currentCell = robotCell;
        while (!currentCell.getRect().contains(goalPoint.coord())){
          path.add(currentCell.makeCenterPoint());
          currentCell = currentCell.toGoalNext;
        }
        path.add(currentCell.makeCenterPoint());
      } else {
        System.out.println("ERROR!: No path calculated.");
      }
      shortPath = shortenPath(path);
      return shortPath;
    }
  }
  
  List<Point2D.Double> shortenPath(List<Point2D.Double> path){
    List<Point2D.Double> shortPath = new ArrayList<Point2D.Double>();
    Point2D.Double goalPoint = path.get(path.size()-1);

    currPoint = path.get(0);
    addPoint = path.get(1);
    checkPoint = path.get(2);

    shortPath.add(currPoint);

    int i;

    
    System.out.println("Entering While");
    while (checkPoint.x!=goalPoint.x || checkPoint.y!=goalPoint.y){//worried
     System.out.println("Currently checking at x: "+ checkPoint.x +", y: " +checkPoint.y);
      if (isVisible(currPoint, checkPoint)){
        i = path.indexOf(checkPoint);
        addPoint = path.get(i);
       // System.out.println("Breaking 1");
        System.out.println(i);
        checkPoint = path.get(i+1);
       // System.out.println("Breaking 1.5");
      } else {
        shortPath.add(addPoint);
        currPoint = addPoint;
        i = path.indexOf(currPoint);
        System.out.println("Breaking 2");
        addPoint = path.get(i+1);
        System.out.println("Breaking 3");
        checkPoint = path.get(i+2); //worried outofbounds
        System.out.println("Breaking 4 "); //shouldnt get here.
      }
    }
    System.out.println("Exiting While");
    shortPath.add(addPoint);
    shortPath.add(checkPoint);
    return shortPath;
  }

  boolean isVisible(Point2D.Double currPoint, Point2D.Double checkPoint){
    double dx = checkPoint.x-currPoint.x;
    double dy = checkPoint.y-currPoint.y;

    System.out.println("Enteredd");
    double oldX = currPoint.x, oldY = currPoint.y;
    for (double j=1.0;j<100.0;j++){
      double newX = currPoint.x+j/100.0*dx;
      double newY = currPoint.y+j/100.0*dy;

      double alfa = Math.atan2((newY-oldY), (newX-oldX));
      if (alfa < 0.0) {
        alfa += Math.PI;
      }
      int alfaInd = (int)((alfa/Math.PI/2.0)*CSObstacle.ANGLE_DIVISIONS);

      System.out.println("AlfaInd=" + alfaInd);
      Point2D.Double newPoint = new Point2D.Double(newX, newY);
      CSGrid.Cell newCell = grid.getCell(new CSCoord(newPoint, alfaInd));

      if (newCell == null || !newCell.free){
        return false;
      }
    }
    return true;

  }

}


