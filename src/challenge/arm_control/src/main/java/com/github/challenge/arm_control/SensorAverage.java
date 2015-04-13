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

package com.github.rosjava.challenge.arm_control;

/**
 * <p>Average a value for a number of ticks. If you find it necessary to filter
 * your sensor data, you may use this example, or write your own.<\p>
 */
class SensorAverage{
  double [] data;
  int data_idx=0;
  double average=0.0;
  int nTicks;

  public SensorAverage(int n) {
    nTicks=n;
    data = new double[nTicks];
  }

  public double step (double sample) {
    average=average+sample/(double)nTicks;
    average=average-data[data_idx]/(double)nTicks;
    data[data_idx]=sample;
    data_idx++;
    if (data_idx==nTicks) data_idx=0;
    return average;
  }

}
