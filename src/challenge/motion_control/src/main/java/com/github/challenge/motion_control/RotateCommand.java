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

public class RotateCommand extends MotionCommand {

  protected double x_prev;
  protected double y_prev;
  protected double theta_prev;

  protected double delta_theta;

  public RotateCommand(double d_theta) {
    delta_theta = d_theta;
  }

  public double getTranslationalVelocity(double x, double y,
                                         double theta) {
    return 0.0;
  }

  public double getRotationalVelocity(double x, double y,
                                      double theta) {
    double delta = calcDeltaAngle(theta);
    System.out.println("theta: " + theta*180.0/Math.PI + " theta_start: " +
                        theta_start*180.0/Math.PI + " delta: " + delta*180.0/Math.PI);
    double w;
    // If within threshold, put a lower rotational velocity.
    if (Math.abs(delta) < Math.PI/180.0*2.0) {
      System.out.println("this choice stop");
      w = 0.0; 
    } else if (Math.abs(delta) < Math.PI/180.0*5.0) {
      System.out.println("this choice /200");
      w = Math.signum(delta)*Math.PI/200.0;
    } else if (Math.abs(delta) < Math.PI/180.0*10.0) {
      w = Math.signum(delta)*Math.PI/50.0;
    } else {
      w = Math.signum(delta)*Math.PI/20.0;
    }
    return w;
  }

  public boolean isDone(double x, double y, double theta) {
    boolean asd=Math.abs(calcDeltaAngle(theta)) < Math.PI/180.0*2.0;;
    System.out.println("Result of done is: " + asd);
    return Math.abs(calcDeltaAngle(theta)) < Math.PI/180.0*2.0;
  }

  protected double calcDeltaAngle(double theta) {
    double delta = (delta_theta - (theta-theta_start))%(2*Math.PI);
    if (delta > 2.0*Math.PI-delta) {
      delta -= 2.0*Math.PI;
    }
    return delta;
  }
}
