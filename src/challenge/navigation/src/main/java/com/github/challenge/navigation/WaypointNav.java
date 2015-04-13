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

import com.github.rosjava.challenge.motion_control.MotionCommand;
import com.github.rosjava.challenge.motion_control.PersistentCommand;
import com.github.rosjava.challenge.motion_control.Robot;
import com.github.rosjava.challenge.motion_control.RotateCommand;
import com.github.rosjava.challenge.motion_control.StopCommand;
import com.github.rosjava.challenge.motion_control.TranslateCommand;
import com.github.rosjava.challenge.motion_control.MotionCommander;

import rss_msgs.OdometryMsg;
import rss_msgs.SonarMsg;
import org.ros.node.ConnectedNode;

enum Robot_State {
	READY, START, ROTATING, ROTATED_STOP, TRANSLATING, TRANSLATED_STOP, DONE
}

public class WaypointNav {
	Robot_State state = Robot_State.READY;
	double delta_dist;
	double delta_theta;
  double heading;
	List<Point2D.Double> waypoints = new ArrayList<Point2D.Double>();

	double robot_x;
	double robot_y;
	double robot_theta;
	Point2D.Double nextpoint;
	MotionCommander motionCommander;
	ListIterator<Point2D.Double> pointsIterator;

	public WaypointNav(Robot_State initState, ConnectedNode initNode, Robot robotPtr) {
		motionCommander = new MotionCommander(initNode, robotPtr);
		this.setState(initState);
	}

	Robot_State getState() {
		return state;
	}

	void setState(Robot_State newState) {
		if (newState != state) {
			System.out.println("Changing state from " + state.toString()
					+ " to " + newState.toString());
			state = newState;
		}
	}

	public double CalcHeading() {
		double delta_y = nextpoint.y - robot_y;
		double delta_x = nextpoint.x - robot_x;
		heading = Math.atan2(delta_y, delta_x);
    heading = heading % (2 * Math.PI); //ensures heading is in [0, 2PI)
		delta_theta = (heading - robot_theta) % (2 * Math.PI);
		return delta_theta;
	}

	public double CalcDistance() {
		double delta_y = nextpoint.y - robot_y;
		double delta_x = nextpoint.x - robot_x;
		this.delta_dist = Math.sqrt(delta_x * delta_x + delta_y * delta_y);
		return this.delta_dist;
	}

	public void resetState() {
		Robot_State state = Robot_State.START;
	}

	public void updatePath(PolygonObstacle polyPath) {
		List<Point2D.Double> vertices = polyPath.getVertices();
		if (vertices == null)
			throw new NullPointerException("No waypoints generated");

		if (state == Robot_State.READY) {
			this.waypoints = vertices;
			this.pointsIterator = waypoints.listIterator();
			this.nextpoint=pointsIterator.next();
			getnextPoint();
			state = Robot_State.START;
		}
	}

	boolean getnextPoint() {
		if (!this.pointsIterator.hasNext()) {
			System.out.println("No more points in Queue");
			return false;
		} else {
			this.nextpoint = this.pointsIterator.next();
			return true;
		}
	}

	boolean isAligned(){
		double deltaTheta = (Math.abs(robot_theta - heading) % (2 * Math.PI)) * 180 / Math.PI; 
		if (deltaTheta <= 1 || deltaTheta >= 359) {
			return true;
    } 
		return false; 
	}

	void handleOdometryMsg(OdometryMsg msg) {
		robot_x = msg.getX();
		robot_y = msg.getY();
		robot_theta = msg.getTheta() % (2 * Math.PI); //limits robotTheta to [0, 2PI)
		motionCommander.step();
		if (this.getState() == Robot_State.START) {
      CalcHeading();
			if(!isAligned()) {
        motionCommander.command(new RotateCommand(this.CalcHeading()));
        this.setState(Robot_State.ROTATING);
			} else {
				this.setState(Robot_State.ROTATED_STOP);
			}
		}
		if (this.getState() == Robot_State.ROTATING) {
			if (motionCommander.isDone()) {
				this.setState(Robot_State.ROTATED_STOP);
				motionCommander.command(new StopCommand());
			} else {
				//motionCommander.command(new RotateCommand(this.CalcHeading()));
      }
		}
		if (this.getState() == Robot_State.ROTATED_STOP){
			if (motionCommander.isDone()) {
				this.setState(Robot_State.TRANSLATING);
				motionCommander.command(new TranslateCommand(this
						.CalcDistance()));
      }
		}

		if (this.getState() == Robot_State.TRANSLATING) {
			if (motionCommander.isDone()) {
				this.setState(Robot_State.TRANSLATED_STOP);
				motionCommander.command(new StopCommand());
			} else {
				//motionCommander.command(new TranslateCommand(this.CalcDistance()));
			}
		}

		if (this.getState() == Robot_State.TRANSLATED_STOP) {
				if (motionCommander.isDone()) {
					if (this.getnextPoint()) {
            motionCommander.command(new RotateCommand(this.CalcHeading()));
						this.setState(Robot_State.ROTATING);
					} else if (!getnextPoint()) {
						this.setState(Robot_State.DONE);
						motionCommander.command(new StopCommand());
						System.out.println("Stopped...not going to move");
					}
			} 
    }
  }

}
