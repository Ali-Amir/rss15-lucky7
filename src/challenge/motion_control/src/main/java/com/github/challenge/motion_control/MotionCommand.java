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

public abstract class MotionCommand {

  protected double x_start;
  protected double y_start;
  protected double theta_start;
  protected double ux;
  protected double uy;

  public void init(double x, double y, double theta) {
    x_start = x;
    y_start = y;
    theta_start = theta;
    ux = Math.cos(theta);
    uy = Math.sin(theta);
  }

  public abstract double getTranslationalVelocity(double x, double y,
                                                  double theta);

  public abstract double getRotationalVelocity(double x, double y,
                                               double theta);

  public abstract boolean isDone(double x, double y, double theta);

}
