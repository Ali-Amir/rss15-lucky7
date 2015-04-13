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

import java.awt.geom.Point2D;
import java.util.ArrayList;

public class ObstacleModel {
  ArrayList<WallSegment>  wallSegments;

  ObstacleModel() {
    wallSegments = new ArrayList<WallSegment>();    
  }

  boolean isObstacleComplete() {
    if (wallSegments.size() < 2) {
      return false;
    }
    double closingDistance =
        wallSegments.get(0).getStartPoint().distance(
            wallSegments.get(wallSegments.size()-1).getEndPoint());
    if (closingDistance < 0.10) {
      return true;
    }
    return false;
  }

  public void addWallSegment(WallSegment wallSegment) {
    System.out.println("Adding a wall segment: " + wallSegment);
    if (wallSegments.size() == 0) {
      wallSegments.add(wallSegment);
      return;
    }

    double dist1 =
        wallSegments.get(0).getStartPoint().distance(
          wallSegment.getStartPoint());
    double dist2 =
        wallSegments.get(0).getStartPoint().distance(
          wallSegment.getEndPoint());
    double dist3 =
        wallSegments.get(wallSegments.size()-1).getEndPoint().distance(
          wallSegment.getStartPoint());
    double dist4 =
        wallSegments.get(wallSegments.size()-1).getEndPoint().distance(
          wallSegment.getEndPoint());
    if (dist1 <= dist2 && dist1 <= dist3 && dist1 <= dist4) {
      wallSegments.add(0, wallSegment.reverse());
    } else if (dist2 <= dist3 && dist2 <= dist4) {
      wallSegments.add(0, wallSegment);
    } else if (dist3 <= dist4) {
      wallSegments.add(wallSegment);
    } else {
      wallSegments.add(wallSegment.reverse());
    }
    System.out.println("============= ADDED A WALL SEGMENT. OBSTACLE IS: ===========");
    for (int i = 0; i < wallSegments.size(); ++i) {
      System.out.print("{"+wallSegments.get(i).getStartPoint() + ","
                       + wallSegments.get(i).getEndPoint() + "} ");
    }
    System.out.println("");
  }

}
