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

public class PersistentCommand extends MotionCommand {

  protected double transVel;
  protected double rotVel;

  public PersistentCommand(double tv, double rv) {
    transVel = tv;
    rotVel = rv;
  }

  public double getTranslationalVelocity(double x, double y,
                                         double theta) {
    return transVel;
  }

  public double getRotationalVelocity(double x, double y,
                                      double theta) {
    return rotVel;
  }

  public boolean isDone(double x, double y, double theta) {
    // TODO
    return false;
  }
}
