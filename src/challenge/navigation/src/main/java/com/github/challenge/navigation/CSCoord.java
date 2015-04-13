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

import java.awt.geom.Point2D;

/**
 * Class that describes C-Space coordinates.
 **/
public class CSCoord {
  Point2D coord;
  protected int thetaInd;

  CSCoord(double _x, double _y, int _thetaInd) {
    coord = new Point2D.Double(_x, _y);
    thetaInd = _thetaInd;
  }

  double x() {
    return coord.getX();
  }

  double y() {
    return coord.getY();
  }

  int thetaInd() {
    return thetaInd;
  }

  Point2D coord() {
    return coord;
  }


  void setX(double _x) {
    coord.setLocation(_x, coord.getY());
  }

  void setY(double _y) {
    coord.setLocation(coord.getX(), _y);
  }

  void setThetaInd(int _thetaInd) {
    thetaInd = _thetaInd;
  }

  void reset(CSCoord p) {
    setX(p.x());
    setY(p.y());
    setThetaInd(p.thetaInd());
  }

  double planarDistanceTo(CSCoord p) {
    return p.coord().distanceSq(coord);
  }

  double distanceTo(CSCoord p) {
    return p.coord().distanceSq(coord) + (p.thetaInd()-thetaInd)*(p.thetaInd-thetaInd);
  }
}
