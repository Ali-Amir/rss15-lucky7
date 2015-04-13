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

/**
 * <p>Class for storing linear wall segments.  Stores the
 * start and end points of the segment, and the parameters
 * of the original linear regression fit<\p>
 */
public class WallSegment {
    // start and end points
    private Point2D startPoint;
    private Point2D endPoint;
    
    // used to define the linear fit: ax+by+c=0
    private double a;
    private double b;
    private double c;

    
    /**
     * <p>Constructs an empty Wall Segment<\p>
     */
    public WallSegment() {
        startPoint = new Point2D.Double();
        endPoint = new Point2D.Double();
    }

    public String toString() {
      return "{" + startPoint.toString() + "," + endPoint.toString() + "}";
    }
    
    /**
     * <p>Constructs a Wall Segment with specified
     * parameters.  Linear fit parameters correspond
     * to the line equation, ax+by+c=0<\p>
     *
     * @param start point of the segment
     * @param end point of the segment
     * @param the linear fit x scale parameter
     * @param the linear fit y scale parameter
     * @param the linear fit offset parameter
     */
    public WallSegment(Point2D startPoint, Point2D endPoint, double a,
                       double b, double c) {
        this();
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        
        this.a = a;
        this.b = b;
        this.c = c;
    }

    public WallSegment reverse() {
      return new WallSegment(endPoint, startPoint, a, b, c);
    }

    /**
     *
     */
    public double getA() {
        return a;
    }

    /**
     *
     */
    public double getB() {
        return b;
    }

    /**
     *
     */
    public double getC() {
        return c;
    }

    /**
     *
     */
    public Point2D getEndPoint() {
        return endPoint;
    }

    /**
     *
     */
    public Point2D getStartPoint() {
        return startPoint;
    }


    /**
     * Return the angle of this segment relative
     * to the x-axis.
     */
    public double getTheta() {
        double delta_x = endPoint.getX() - startPoint.getX();
        double delta_y = endPoint.getY() - startPoint.getY();
        return Math.atan2(delta_y, delta_x);
    }
    
}


