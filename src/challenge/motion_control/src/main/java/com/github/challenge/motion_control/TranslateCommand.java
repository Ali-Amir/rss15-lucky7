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

public class TranslateCommand extends MotionCommand {

  protected double delta_d;
  protected double ux;
  protected double uy;

  public TranslateCommand(double d_d) {
    delta_d = d_d;
  }

  public double getTranslationalVelocity(double x, double y,
                                         double theta) {
    if (isDone(x, y, theta)) {
      return 0.0;
    }

    double delta_cur = (x - x_start)*ux + (y - y_start)*uy;
    double v = (delta_d - delta_cur)*2.0;
    v = Math.min(v, 0.03);
    v = Math.max(v, -0.03);
    return v;
  }

  public double getRotationalVelocity(double x, double y,
                                      double theta) {
    return 0.0;
  }

  public boolean isDone(double x, double y, double theta) {
    return Math.abs(Math.sqrt((x - x_start)*(x - x_start) +
                    (y - y_start)*(y - y_start)) - Math.abs(delta_d)) < 0.05;
  }
}
