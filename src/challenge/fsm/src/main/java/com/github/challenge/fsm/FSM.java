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

package com.github.rosjava.challenge.fsm;

import java.awt.geom.Point2D;
import java.util.*;
import java.io.FileReader;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.AbstractNodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.node.parameter.ParameterTree;

import rss_msgs.ArmMsg;
import rss_msgs.BumpMsg;
import rss_msgs.MotionMsg;
import rss_msgs.OdometryMsg;
import rss_msgs.GraspingMsg;
import rss_msgs.RobotLocation;
import rss_msgs.NavStatus;

import com.github.rosjava.challenge.vision.BlobTracking;
import com.github.rosjava.challenge.gui.Image;

import com.github.rosjava.challenge.arm_control.Grasping;


enum RobotFSM{

	INITIALIZE,

	EXPLORATORY_PATHING,

	SMART_PATHING,

	COLLECTION,

	RETURN_TO_ASSEMBLY,

	ASSEMBLY

}

public class FSM extends AbstractNodeMain {
	//subscribes to rss/ArmStatus, rss/DigitalIO, rss/Odometry

	/**
	 * <p>Stores the current FSM state<\p>
	 */
	RobotFSM fsmState;

	/**
	 * <p>Starting pose of robot before moving<\p>
	 */
	Point2D.Double currentPoint = new Point2D.Double();

	double currentTheta;

	/**
	 * <p>Starting pose of robot before moving<\p>
	 */
	Point2D.Double startPoint;

	Point2D.Double assemblyPoint = new Point2D.Double();
	Point2D.Double testPoint = new Point2D.Double(3.34, 0.73);
  int queueInd = 0;



	/**
	 * <p>Target pose for robot at end of motion<\p>
	 */
	Point2D.Double targetPoint;

	private ArrayList<Point2D.Double> blockLocations = new ArrayList<Point2D.Double>();
	private boolean testBlockLocations = true;

	private Publisher<MotionMsg> motionPub;
	private Publisher<sensor_msgs.Image> vidPub;

	private Subscriber<RobotLocation> odoSub;
	//ALI!
	//private Subscriber<LocalizationMsg> locSub;

	private Publisher<RobotLocation> navPub;
	private Publisher<GraspingMsg> graspingPub;

	//private Subscriber<NavMsg> navSub;
	private Subscriber<GraspingMsg> graspingSub;
	//END ALI!

	private double targetTheta;
	private int blockCount = 0;
  protected LinkedList<Block> blocks = new LinkedList<Block>();

  protected Block[] queue = {
    new Block(0.6, 0.6, "asd"),
    new Block(2.3875, 1.1625, "asd"),
    new Block(3.9125, 0.525, "asd"),
    new Block(4.25, 2.5875, "asd"),
    new Block(0.9375, 2.1, "asd"),
  };


	/**
	 * <p>Constructor for Grasping object<\p>
	 */
	public FSM() {
		fsmState = RobotFSM.INITIALIZE;
		System.out.println("FSM: FSM Initialized");

	}

	

  private class Block {
    public double x;
    public double y;
    public String color;
    Block(double x_, double y_, String color_) {
      x = x_;
      y = y_;
      color = color_;
    }
  }
	

	@Override
	public void onStart(ConnectedNode node){
		motionPub = node.newPublisher("command/Motors", MotionMsg._TYPE);
		vidPub = node.newPublisher("/rss/blobVideo", sensor_msgs.Image._TYPE);

		odoSub = node.newSubscriber("localization/update", RobotLocation._TYPE);
		odoSub.addMessageListener(new OdometryListener(this));

		//ALI!
		graspingPub = node.newPublisher("command/Grasping", GraspingMsg._TYPE);
		navPub = node.newPublisher("navigation/GoTo", RobotLocation._TYPE);

		//navSub = node.newSubscriber("rss/NavStatus", NavMsg._TYPE);
		//navSub.addMessageListener(new NavListener(this));

		graspingSub = node.newSubscriber("rss/GraspingStatus", GraspingMsg._TYPE);
		graspingSub.addMessageListener(new GraspingListener(this));

		//locSub = node.newSubscriber("module/Localization", LocalizationMsg._TYPE);
		//locSub.addMessageListener(new LocalizationListener(this));
		//END ALI!
    
    ParameterTree paramTree = node.getParameterTree();    
    String mapFileName = paramTree.getString(node.resolveName("~/mapFileName"));
    readAndInitBlocks(mapFileName);
	}

	@Override public GraphName getDefaultNodeName() {
		return GraphName.of("rss/FSM");
	}

  protected void readAndInitBlocks(String mapFileName) {
    try {
      FileReader f = new FileReader(mapFileName);
      Scanner scan = new Scanner(f); 
      // Read start/goal points and world boundaries
      scan.nextDouble(); scan.nextDouble(); 
      scan.nextDouble(); scan.nextDouble(); 
      scan.nextDouble(); scan.nextDouble(); scan.nextDouble(); scan.nextDouble(); 
      // Read obstacles
      int numObs = scan.nextInt();
      for (int i = 0; i < numObs; ++i) {
        int numVer = scan.nextInt();
        for (int j = 0; j < numVer; ++j) {
          scan.nextDouble(); scan.nextDouble();
        }
      }
      int numBlocks = scan.nextInt();
      for (int i = 0; i < numBlocks; ++i) {
        double x, y;
        x = scan.nextDouble();
        y = scan.nextDouble();
        String color;
        color = scan.next();
        addBlock(new Block(x, y, color));
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  protected void addBlock(Block block) {
    blocks.add(block);
  }

	/**
	 * <p>Handle an OdometryMessage<\p>
	 */
	public void handle(RobotLocation msg) {
		//System.out.println("Odometry Message: (" + msg.getX() + " ," + msg.getY() + ", " + msg.getTheta() + ")");
		currentPoint.x = msg.getX();
		currentPoint.y = msg.getY();
		currentTheta = msg.getTheta();

    synchronized(fsmState) {
		switch (fsmState) {

			case INITIALIZE: {
				setVelocity(0.0, 0.0);

				// fsmState = RobotFSM.SMART_PATHING;
				// setNavigation(testPoint);
				break;
			}

			case EXPLORATORY_PATHING: {
				break;
			}

			case SMART_PATHING: {
        double distance = Math.sqrt(
            (currentPoint.x-queue[queueInd].x)*(currentPoint.x-queue[queueInd].x) +
            (currentPoint.y-queue[queueInd].y)*(currentPoint.y-queue[queueInd].y));
        if (distance < 2e-1) {
          queueInd = (queueInd + 1) % queue.length;
        }
        Point2D.Double tarPoint = new Point2D.Double(queue[queueInd].x, queue[queueInd].y);
        setNavigation(tarPoint);
        /*
				double dx = Math.abs(currentPoint.x-testPoint.x);
				double dy = Math.abs(currentPoint.y-testPoint.y);
				double dist = Math.sqrt(dx*dx + dy*dy);
				if (dist<.03){
					fsmState = RobotFSM.COLLECTION;
					setGrasping(COLLECTING);
				}
        */
			}

			case COLLECTION: {
				break;
			}

			case ASSEMBLY: {
				break;
			}
		}
    }
	}

	
	/**
	 * <p> Handle an Grasping Message<\p>
	 */

	static final int GRASPING_INITIALIZED = -1;
	static final int COLLECTING = 0;
	static final int ASSEMBLING = 1;
  static final int BACKGROUND_PROCESSING = 2;
  static final int WALL_COLLECTING = 3;
	static final int OFF = 5;

	public void handle(GraspingMsg msg){
		//System.out.println("FSM: //GOT GRASPING MESSAGE//");


		int mode = msg.getServomode();
		boolean notRisky = msg.getCollected();
		boolean found = msg.getFound();


		switch (fsmState) {

			case INITIALIZE: {
				fsmState = RobotFSM.SMART_PATHING;
				System.out.println("FSM: //COLLECTION//");
				setGrasping(BACKGROUND_PROCESSING);
				break;
			}

			case EXPLORATORY_PATHING: {
				switch (mode) {
					case COLLECTING: {
						fsmState = RobotFSM.COLLECTION;
						stopNavigation();
						if (notRisky){
							setGrasping(COLLECTING);
						} else {
							setGrasping(WALL_COLLECTING);
						}
						break;
					}
				}
				break;
			}

			case SMART_PATHING: {
				switch (mode) {
					case COLLECTING: {
						fsmState = RobotFSM.COLLECTION;
						stopNavigation();
						if (notRisky){
							setGrasping(COLLECTING);
						} else {
							setGrasping(WALL_COLLECTING);
						}
						break;
					}
				}
				break;
			}

			case COLLECTION: {
				switch (mode) {

					case COLLECTING: {
						break;
					}

					case ASSEMBLING: {
						break;
					}

					case OFF: {

						if (notRisky) {

							System.out.println("FSM: BLOCK COLLECTED");

							//blockCount += 1;

							if (blockCount>4){
								fsmState = RobotFSM.RETURN_TO_ASSEMBLY;
							} else {
								fsmState = RobotFSM.SMART_PATHING;
							}

						
						} else {
							System.out.println("FSM: BLOCK NOT COLLECTED");
						}

						// if (collected){
						// 	startPoint = currentPoint;
						// 	startTheta = currentTheta;
						// 	targetPoint = new Point2D.Double();
						// 	targetPoint.x = startPoint.x - 
						// 	.5*Math.cos(startTheta);
						// 	targetPoint.y = startPoint.y -
						// 	.5*Math.sin(startTheta);
						// 	targetTheta = startTheta;

						// 	setNavigationPoints(startPoint, targetPoint);

						// 	fsmState = RobotFSM.SMART_PATHING;
						// }
						break;
						
					}
				}

				break;
			}

			case ASSEMBLY: {
				switch (mode) {

					case ASSEMBLING: {
						System.out.println("Starting to assembly blocks");
						break;
					}
					case OFF:{
					System.out.println("Assembly done");
					fsmState=RobotFSM.SMART_PATHING;
					break;
					}
				}
			}
		}
	}

	// /**
	//  * <p> Handle an Navigation Message<\p>
	//  */

	// static final int NAV_INITIALIZED = 0;
	// static final int IN_PROGRESS = 1;
	// static final int FINISHED = 2;

	// public void handle(NavStatus msg){

	// 	int status = msg.getStatus();

	// 	switch (fsmState) {

	// 		case INITIALIZE: {
	// 			// if (status=="Initialized"){
	// 			// 	startPoint = currentPoint;
	// 			// 	startTheta = currentTheta;
	// 			// 	targetPoint = new Point2D.Double();
	// 			// 	targetPoint.x = startPoint.x + 
	// 			// 	.5*Math.cos(startTheta);
	// 			// 	targetPoint.y = startPoint.y +
	// 			// 	.5*Math.sin(startTheta);
	// 			// 	targetTheta = startTheta;

	// 			// 	setNavigationPoints(startPoint, targetPoint);

	// 			// 	fsmState = RobotFSM.SMART_PATHING;
	// 			// }
	// 			break;
	// 		}

	// 		case EXPLORATORY_PATHING: {
	// 			break;
	// 		}

	// 		case SMART_PATHING: {

	// 			switch(status) {

	// 				case IN_PROGRESS: {
	// 					break;
	// 				}

	// 				case FINISHED: {
	// 					double thetaOffset = currentTheta - targetTheta;

	// 					double rotVel;
						
	// 					if (thetaOffset<0){
	// 						rotVel = .5;
	// 					}
	// 					else{
	// 						rotVel = .5;
	// 					}

	// 					if (Math.abs(thetaOffset)<thetaThreshold){
	// 						fsmState = RobotFSM.COLLECTION;
	// 						setGrasping(COLLECTING);
	// 					} else {
	// 						setVelocity(rotVel, 0);
	// 					}
	// 				}
	// 			}
	// 		}

	// 		case COLLECTION: {
	// 			break;
	// 		}

	// 		case ASSEMBLY: {
	// 			break;
	// 		}
	// 	}
	// }

	public void setVelocity(double rotVel, double transVel) {
		MotionMsg motionMsg = motionPub.newMessage();
		motionMsg.setRotationalVelocity(rotVel);
		motionMsg.setTranslationalVelocity(transVel);
		motionPub.publish(motionMsg);
	}

	public void setNavigation(Point2D.Double targetPoint){
		RobotLocation navMsg = navPub.newMessage();
		navMsg.setX(targetPoint.getX());
		navMsg.setY(targetPoint.getY());
		navMsg.setTheta(-20.0);
	 	navPub.publish(navMsg);
	}

	public void stopNavigation(){
		RobotLocation navMsg = navPub.newMessage();
		navMsg.setTheta(20.0);
	 	navPub.publish(navMsg);
	}

	public void setGrasping(int graspingMode){
		GraspingMsg graspingMsg = graspingPub.newMessage();
		graspingMsg.setServomode(graspingMode);
		graspingPub.publish(graspingMsg);
	}

}
