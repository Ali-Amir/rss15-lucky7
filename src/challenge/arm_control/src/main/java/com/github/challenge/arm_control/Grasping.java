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

import java.awt.geom.Point2D;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.AbstractNodeMain;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import rss_msgs.ArmMsg;
import rss_msgs.BumpMsg;
import rss_msgs.MotionMsg;
import rss_msgs.OdometryMsg;
import rss_msgs.GraspingMsg;
import rss_msgs.LocFree;
import rss_msgs.LocFreeRequest;
import rss_msgs.LocFreeResponse;
import rss_msgs.RobotLocation;

//import com.github.rosjava.challenge.fsm.GraspingListener;

import com.github.rosjava.challenge.vision.BlobTracking;
import com.github.rosjava.challenge.gui.Image;

/* This uses two modes: with vision, without.
 * Without-vision will grasp upon breakbeam and dump the object in the back
 * of the robot (Only when the arm is extended).
 *
 * With-vision will be flagged when the object has been visually servoed to
 * the correct pixel coordinate location. The arm is then extended and the
 * without-vision routine can execute.
 * *
 **/

enum RoboFSM {
	/**
	 * <p>FSM state: arm initial position.  The initial position is expected in
	 * fully retracted state, shoulder back, arm back, gripper closed.<\p>
	 */
	INITIALIZE_ARM,
	/**
	 * <p>FSM state: set arm to gated position
	 * Shoulder: -pi/2
	 * Wrist: pi/2<\p>
	 */
	SET_ARM_TO_GATE,
	/**
	 * <p>FSM state: maintain gated position<\p>
	 */
	SET_ARM_TO_PULL,
	ENGAGE_BLOCK,
	RELEASE_BLOCK,
	/**
	 * <p>FSM state: set blade to collect block<\p>
	 */
	SET_ARM_TO_COLLECT,
	/**
	 * <p>FSM state: collecting motion<\p>
	 */
	COLLECTING,
	/**
	 * <p>FSM state: object has been dropped<\p>
	 */

	VISUAL_SERVO_APPROACH,

	VISUAL_SERVO_SEARCH,

  	RELEASING,

  	MOVE_FORWARD,
  	MOVE_BACKWARD,
  	ASSEMBLY_ROTATE,
  	BLIND_APPROACH,

  	REAPPROACH,
  	PULL_BACK,
  	SET_ARM_RETRACTED,
  	BACKGROUND_PROCESSING_STATE,

  	OFF

}

public class Grasping extends AbstractNodeMain {
	//subscribes to rss/ArmStatus, rss/DigitalIO, rss/Odometry

	/**
	 * <p>Used to cleanly separate code fragments from different Parts of the
	 * lab<\p>
	 */
	public int SERVO_MODE;
  protected static final double VS_TIMEOUT_SEC = 1.0;
  protected double curLocX;
  protected double curLocY;
  protected double curLocTheta;
  protected double prevRotVel;
  protected double prevTransVel;
  protected double lastDetectionTime;
	static final int INITIALIZED = -1;
	static final int COLLECTING = 0;
	static final int ASSEMBLING = 1;
	static final int BACKGROUND_PROCESSING =2 ;

	static final int WALL_COLLECTING = 3;

	static final int OFF = 5;
	/**
	 * <p>Shoulder joint array index.  Note, current sense is on Orc port 0,
	 * which we want for shoulder.<\p>
	 */
	static final int SHOULDER_INDEX = 0;

	/**
	 * <p>Wrist joint array index<\p>
	 */
	static final int WRIST_INDEX = 1;

	/**
	 * <p>Shoulder controller instance<\p>
	 */
	ShoulderController shoulderControl = new ShoulderController();

	/**
	 * <p>Wrist controller instance<\p>
	 */
	WristController wristControl = new WristController();

	/**
	 * <p>Entire Arm controller, array of JointControllers<\p>
	 */
	JointController[] armControl = {shoulderControl, wristControl};

	/**
	 * <p>First time stamp<\p>
	 */
	public double firstTime = -1;

	/**
	 * <p>Stores the time in last controller step iteration<\p>
	 */
	public double lastTime = -1;

	/**
	 * <p>Stores the cycle time between subsequent iterations<\p>
	 */
	public double cycleTime = -1;

	/**
	 * <p>Commanded servo positions to desired set point<\p>
	 */
	public long[] armTheta = new long[]{0,0,0,0,0,0,0,0};

	/**
	 * <p>Stores the current FSM state<\p>
	 */
	RoboFSM fsmState;

	static final double EPS_SEARCH_STANDOFF = 0.005; // m
	static final double SEARCH_STANDOFF = 0.5; // m
	static final double APPROACH_STANDOFF = 0.31;

	/**
	 * <p>Indicates first iteration through position controller<\p>
	 */
	boolean startingMove = true;
boolean rotating = true;
	/**
	 * <p> Indicate whether the block is collected or not <\p>
	 */
	boolean blockCollected = true;
	boolean blockFound;

/**
	 * <p> Indicate whether the structure is assembled or not <\p>
	 */
	boolean assembled = true;
	
	/**
	 * <p>Starting pose of robot before moving<\p>
	 */
	Point2D.Double startPoint;

	/**
	 * <p>Target pose for robot at end of motion<\p>
	 */
	Point2D.Double targetPoint;

	double moveDistance;

	/**
	 * <p>Distance to transport object (m)<\p>
	 */
	static final double TRANSPORT_DISTANCE = 0.5;

	static final double BACK_DISTANCE = 0.65;
	
	static final double APPROACH_DISTANCE = 0.15;



	/**
	 * <p>Translational velocity while moving (m/s).</p>
	 **/
	public static final double WHEEL_TV = 0.12;

	/**
	 * <p>Proportional gain for rotation controller while moving.</p>
	 **/
	public static final double WHEEL_RV_GAIN = .8;

	/**
	 * <p>Max rotational velocity while moving (rad/s).</p>
	 **/
	public static final double WHEEL_MAX_RV = 0.1;

	/**
	 * <p>Target reached threshold (m).</p>
	 **/
	public static final double TARGET_THRESHOLD = 0.02;

	/**
	 * <p>Target reached threshold (m).</p>
	 **/
	public static final double THETA_THRESHOLD = 2.0*Math.PI/180;

	/**
	 * <p>Indicates backward direction motion<\p>
	 */
	public static final int DIR_BACKWARD = -1;

	/**
	 * <p>Indicates forward direction motion<\p>
	 */
	public static final int DIR_FORWARD = 1;
	private static final double BLIND_FORWARD_VELOCITY = .1;

	/**
	 * <p>The blob tracker<\p>
	 */
	private BlobTracking blobTrack = null;

	private Publisher<ArmMsg> armPub;
	private Publisher<MotionMsg> motionPub;
	private Publisher<sensor_msgs.Image> vidPub;
	private double startTheta;
	private double targetTheta;
	private boolean bumpPressed;

	private Publisher<GraspingMsg> graspingPub;
	private Subscriber<GraspingMsg> graspingSub;
	private Subscriber<RobotLocation> locSub;

	/**
	 * <p>Constructor for Grasping object<\p>
	 */
	public Grasping() {

		SERVO_MODE = INITIALIZED;//PART_2A;
		//System.out.println("FIRST MODE"); 
		fsmState = RoboFSM.INITIALIZE_ARM;
		System.out.println("GRASPING: Grasping class initialized");
	}

	private Subscriber<ArmMsg> armSub;
	private Subscriber<BumpMsg> bumpSub;
	//private Subscriber<OdometryMsg> odoSub;
	private Subscriber<sensor_msgs.Image> vidSub;
  private ServiceClient<LocFreeRequest, LocFreeResponse> freeCellClient;
	@Override
	public void onStart(ConnectedNode node){
    while (true) {
      try {
        freeCellClient = 
          node.newServiceClient("navigation/IsLocationFree", LocFree._TYPE);
        break;
      } catch (Exception e) {
        //System.err.println("No SERVICE IsLocationFree found!!!!!!");
      }
    }
    System.out.println("Connected to islocationfree service!");

		armPub = node.newPublisher("command/Arm", ArmMsg._TYPE);
		motionPub = node.newPublisher("command/Motors", MotionMsg._TYPE);
		vidPub = node.newPublisher("/rss/blobVideo", sensor_msgs.Image._TYPE);
		graspingPub = node.newPublisher("rss/GraspingStatus", GraspingMsg._TYPE);

    curLocX = 0.6;
    curLocY = 0.6;
    curLocTheta = 0.0;

		armSub = node.newSubscriber("rss/ArmStatus", ArmMsg._TYPE);
		armSub.addMessageListener(new ArmListener(this));
		bumpSub = node.newSubscriber("rss/BumpSensors", BumpMsg._TYPE);
		bumpSub.addMessageListener(new BumpListener(this));
    /*
		odoSub = node.newSubscriber("rss/odometry", OdometryMsg._TYPE);
		odoSub.addMessageListener(new OdometryListener(this));
    */
		locSub = node.newSubscriber("localization/update", RobotLocation._TYPE);
		locSub
		.addMessageListener(new MessageListener<RobotLocation>() {
			@Override
			public void onNewMessage(RobotLocation message) {
        curLocX = message.getX();
        curLocY = message.getY();
        curLocTheta = message.getTheta();
        if (blobTrack != null) {
          blobTrack.updateLocation(curLocX, curLocY, curLocTheta);
        }
        handle(message);
			}
		});
		graspingSub = node.newSubscriber("command/Grasping", GraspingMsg._TYPE);
		graspingSub.addMessageListener(new GraspingListener(this));

        final boolean reverseRGB = node.getParameterTree().getBoolean("reverse_rgb", false);

		vidSub = node.newSubscriber("/rss/video", sensor_msgs.Image._TYPE);
		vidSub.addMessageListener(new MessageListener<sensor_msgs.Image>() {
			@Override
			public void onNewMessage(sensor_msgs.Image message) {
				byte[] rgbData;

				if (reverseRGB) {
					rgbData = Image.RGB2BGR(message.getData().array(),
							(int) message.getWidth(), (int) message.getHeight());
				}
				else {
					rgbData = message.getData().array();
				}

				byte[] newData = new byte[message.getWidth()*message.getHeight()*3];

				int offset = 18;

				for (int y = 0; y<message.getHeight();y++){
					for (int x=0; x<message.getWidth(); x++){

						int new_x = x - offset;
						if (new_x<0){
							new_x = new_x + message.getWidth();
						}
						int old_index = (y*message.getWidth() + x) * 3; 
						int new_index = (y*message.getWidth() + new_x)*3;

						newData[new_index] = rgbData[old_index];
						newData[new_index+1] = rgbData[old_index+1];
						newData[new_index+2] = rgbData[old_index+2];


					}
				}

				handle(newData, (int)message.getWidth(), (int)message.getHeight());
			}
		});
	}

	@Override public GraphName getDefaultNodeName() {
		return GraphName.of("rss/grasping");
	}

	/** 
	 * <p> Handle a Grasping Message <\p>
	 */

	public void handle(GraspingMsg msg){
		SERVO_MODE = msg.getServomode();

		if (SERVO_MODE == COLLECTING  || SERVO_MODE==ASSEMBLING || SERVO_MODE == WALL_COLLECTING){
			fsmState = RoboFSM.INITIALIZE_ARM;
		}
		else if (SERVO_MODE == BACKGROUND_PROCESSING){
			fsmState = RoboFSM.BACKGROUND_PROCESSING_STATE;
		}


	}

	/**
	 * <p>Handle an ArmMessage<\p>
	 */
  	int counter = 0;
	public void handle(ArmMsg msg) {
		if (SERVO_MODE==INITIALIZED){
			setGrasping(INITIALIZED, false, false);
		}

		wristControl.update(msg.getPwms()[WRIST_INDEX]);
		shoulderControl.update(msg.getPwms()[SHOULDER_INDEX]);

	    ++counter;
	    if (counter % 1 != 0) {
	      return;
	    }

		armTheta[WRIST_INDEX] = wristControl.step(msg);
		armTheta[SHOULDER_INDEX] = shoulderControl.step(msg);

  		// Send arm pose to the Orc
		ArmMsg armMsg = armPub.newMessage();
		armMsg.setPwms(armTheta);
		armPub.publish(armMsg);

		if (SERVO_MODE == COLLECTING) {

			switch (fsmState) {

				case INITIALIZE_ARM: {
					if (wristControl.isAtDesired() && shoulderControl.isAtDesired() ) {
						System.out.println("========================================================");
						System.out.println("GRASPING: Arm is now initialized (in retracted state)");
						

						// part 3b
						fsmState = RoboFSM.VISUAL_SERVO_SEARCH;
						//fsmState = RoboFSM.SET_ARM_FOR_GRASP;
						// Part 4:
					}
					break;
				}

				case SET_ARM_TO_COLLECT: {
					System.out.println("GRASPING: SET_BLADE_TO_COLLECT");
					if (wristControl.isAtDesired() && shoulderControl.isAtDesired()) {
						System.out.println("GRASPING: BLADE IS SET TO COLLECT");
						fsmState = RoboFSM.MOVE_FORWARD;
						moveDistance = 0.35;
						//fsmState = RoboFSM.BLIND_APPROACH;
					}
					break;
				}

				case COLLECTING: {
					// if (bumpPressed){
					// 	blockCollected = true;
					// 	//code here to announce the block has been collected.
					// }
					System.out.println("GRASPING: COLLECTING");
					if (wristControl.isAtDesired() && shoulderControl.isAtDesired()) {
						
     					//BUMP SENSOR LOGIC
						// try {
						//     Thread.sleep(1000);                 //1000 milliseconds is one second.
						// } catch(InterruptedException ex) {
						//     Thread.currentThread().interrupt();
						// }
						//BUMP SENSOR LOGIC
						
						// if (bumpPressed){
						// 	System.out.println("GRASPING: BLOCK IS COLLECTED");
						// 	fsmState = RoboFSM.OFF;
						// 	setGrasping(OFF, true);
						// } else {
						// 	System.out.println("GRASPING: BLOCK WAS NOT COLLECTED");
						// 	fsmState = RoboFSM.OFF;
						// 	setGrasping(OFF, false);
						// }

						//fsmState = RoboFSM.INITIALIZE_ARM;

						fsmState = RoboFSM.OFF;

						setGrasping(OFF, true, true);
							//setGrasping(OFF, true); //fsmState = RoboFSM.BLIND_APPROACH;
					}
					break;
				}

				case RELEASING: {
					System.out.println("GRASPING: RELEASING");
					if (wristControl.isAtDesired() && shoulderControl.isAtDesired()) {
						System.out.println("GRASPING: BLOCK IS RELEASED");
						fsmState = RoboFSM.OFF;
						setGrasping(OFF, true, true);
						//fsmState = RoboFSM.BLIND_APPROACH;
					}
					break;
				}
			}

		} else if (SERVO_MODE == WALL_COLLECTING) {

			switch (fsmState) {

				case INITIALIZE_ARM: {
					if (wristControl.isAtDesired() && shoulderControl.isAtDesired() ) {
						System.out.println("========================================================");
						System.out.println("GRASPING: Arm is now initialized (in retracted state)");
						

						// part 3b
						fsmState = RoboFSM.VISUAL_SERVO_SEARCH;
						//fsmState = RoboFSM.SET_ARM_FOR_GRASP;
						// Part 4:
					}
					break;
				}

				case SET_ARM_TO_PULL: {
					//System.out.println("GRASPING: SET_BLADE_TO_PULL");
					if (wristControl.isAtDesired() && shoulderControl.isAtDesired()) {
						System.out.println("GRASPING: BLADE IS SET TO COLLECT");
						//fsmState = RoboFSM.ENGAGE_BLOCK;<<
						fsmState = RoboFSM.BLIND_APPROACH;
						moveDistance = .11;
					}
					break;
				}

				case SET_ARM_TO_COLLECT: {
					System.out.println("GRASPING: SET_BLADE_TO_COLLECT");
					if (wristControl.isAtDesired() && shoulderControl.isAtDesired()) {
						System.out.println("GRASPING: BLADE IS SET TO COLLECT");
						fsmState = RoboFSM.MOVE_FORWARD;
						moveDistance = 0.18;
						//fsmState = RoboFSM.BLIND_APPROACH;
					}
					break;
				}

				case SET_ARM_RETRACTED: {
					System.out.println("GRASPING: SET_BLADE_TO_COLLECT");
					if (wristControl.isAtDesired() && shoulderControl.isAtDesired()) {
						System.out.println("GRASPING: BLADE IS SET TO COLLECT");
						fsmState = RoboFSM.OFF;
						//fsmState = RoboFSM.BLIND_APPROACH;
					}
					break;
				}

				case ENGAGE_BLOCK: {
					System.out.println("GRASPING: ENGAGE_BLOCK");
					if (wristControl.isAtDesired() && shoulderControl.isAtDesired()) {
						System.out.println("GRASPING: BLADE IS SET TO COLLECT");
						fsmState = RoboFSM.PULL_BACK;
						moveDistance = 0.2;

						//fsmState = RoboFSM.BLIND_APPROACH;
					}
					break;
				}

				case RELEASE_BLOCK: {
					System.out.println("GRASPING: ENGAGE_BLOCK");
					if (wristControl.isAtDesired() && shoulderControl.isAtDesired()) {
						System.out.println("GRASPING: BLADE IS SET TO COLLECT");
						fsmState = RoboFSM.SET_ARM_TO_COLLECT;

						//fsmState = RoboFSM.BLIND_APPROACH;
					}
					break;
				}

				case COLLECTING: {
					// if (bumpPressed){
					// 	blockCollected = true;
					// 	//code here to announce the block has been collected.
					// }
					System.out.println("GRASPING: COLLECTING");
					if (wristControl.isAtDesired() && shoulderControl.isAtDesired()) {
						
     					//BUMP SENSOR LOGIC
						// try {
						//     Thread.sleep(1000);                 //1000 milliseconds is one second.
						// } catch(InterruptedException ex) {
						//     Thread.currentThread().interrupt();
						// }
						//BUMP SENSOR LOGIC
						
						// if (bumpPressed){
						// 	System.out.println("GRASPING: BLOCK IS COLLECTED");
						// 	fsmState = RoboFSM.OFF;
						// 	setGrasping(OFF, true);
						// } else {
						// 	System.out.println("GRASPING: BLOCK WAS NOT COLLECTED");
						// 	fsmState = RoboFSM.OFF;
						// 	setGrasping(OFF, false);
						// }

						//fsmState = RoboFSM.INITIALIZE_ARM;

						fsmState = RoboFSM.OFF;

						setGrasping(OFF, true, true);
            //setGrasping(OFF, true);
						//fsmState = RoboFSM.BLIND_APPROACH;
					}
					break;
				}

				case RELEASING: {
					System.out.println("GRASPING: RELEASING");
					if (wristControl.isAtDesired() && shoulderControl.isAtDesired()) {
						System.out.println("GRASPING: BLOCK IS RELEASED");
						fsmState = RoboFSM.OFF;
						setGrasping(OFF, true, true);
						//fsmState = RoboFSM.BLIND_APPROACH;
					}
					break;
				}
			}

		} else if (SERVO_MODE == ASSEMBLING) {

			switch (fsmState) {
				
				case INITIALIZE_ARM: {
					if (wristControl.isAtDesired() && shoulderControl.isAtDesired() ) {
						System.out.println("========================================================");
						System.out.println("GRASPING: Arm is now initialized (in retracted state)");

						// part 3b
						fsmState = RoboFSM.SET_ARM_RETRACTED;
					}
					break;
				}

				case SET_ARM_TO_GATE: {
					if (wristControl.isAtDesired() && shoulderControl.isAtDesired() ) {
						System.out.println("========================================================");
						System.out.println("GRASPING: Arm is now initialized (in retracted state)");

					}
					break;
				}

				case SET_ARM_RETRACTED: {
					if (wristControl.isAtDesired() && shoulderControl.isAtDesired() ) {
						System.out.println("========================================================");
						System.out.println("ASSEMBLING: Arm is now initialized to releasing state");
						fsmState = RoboFSM.MOVE_BACKWARD;
					}
					break;
				} 
				case OFF: {
					System.out.println("Structure Assembly finished"); 

       			    fsmState = RoboFSM.OFF;
					setGrasping(OFF, false, false);
					break;	
				}
					
			}
		} 
	}
	/**
	 * <p>Handle an OdometryMessage<\p>
	 */
	public void handle(RobotLocation msg) {
		//System.out.println("Odometry Message: (" + msg.getX() + " ," + msg.getY() + ", " + msg.getTheta() + ")");

		// 
		if (SERVO_MODE == COLLECTING) {
			switch (fsmState) {

				case INITIALIZE_ARM: {
					//setVelocity(0.0, 0.0);
					break;
				}

				case MOVE_FORWARD: {

					//System.out.println("GRASPING: MOVE FORWARD");
					// TODO
					//if (Math.abs(blobTrack.target
					// check distance to target and decrease standoff

					// if object is lost, go back to VSSEARCH

					//this is just a placeholder for moving forward.
					System.out.println("GRASPING: *** MOVE_FORWARD *** " + startingMove);
					if(startingMove) {
						startPoint = new Point2D.Double();
						startPoint.x = msg.getX();
						startPoint.y = msg.getY();
						startTheta = msg.getTheta();
						targetPoint = new Point2D.Double();
						targetPoint.x = startPoint.x +
						moveDistance*Math.cos(startTheta);
						targetPoint.y = startPoint.y +
						moveDistance*Math.sin(startTheta);
						targetTheta = startTheta;
						startingMove = false;
					}

					if(moveTowardTarget(msg.getX(), msg.getY(), msg.getTheta(), targetPoint.x,
							targetPoint.y, startTheta, -0.10, 0.03)) {
						System.out.println("GRASPING: We are within range of target");
						// TBD
						//(new GUIPointMessage(tX, tY, MapGUI.X_POINT)).publish();
						startingMove = true;
						setVelocity(0.0, 0.0);
						//					Robot.setVelocity(0.0, 0.0);
						fsmState = RoboFSM.COLLECTING; 
					}
					break;
				}
			}
		} else if (SERVO_MODE == WALL_COLLECTING) {
			switch (fsmState) {

				case INITIALIZE_ARM: {
					//setVelocity(0.0, 0.0);
					break;
				}

				case MOVE_FORWARD: {

					//System.out.println("GRASPING: MOVE FORWARD");
					// TODO
					//if (Math.abs(blobTrack.target
					// check distance to target and decrease standoff

					// if object is lost, go back to VSSEARCH

					//this is just a placeholder for moving forward.
          // TODO: logging
					//System.out.println("GRASPING: *** MOVE_FORWARD *** " + startingMove);
					if(startingMove) {
						startPoint = new Point2D.Double();
						startPoint.x = msg.getX();
						startPoint.y = msg.getY();
						startTheta = msg.getTheta();
						targetPoint = new Point2D.Double();
						targetPoint.x = startPoint.x +
						moveDistance*Math.cos(startTheta);
						targetPoint.y = startPoint.y +
						moveDistance*Math.sin(startTheta);
						targetTheta = startTheta;
						startingMove = false;
					}

					if(moveTowardTarget(msg.getX(), msg.getY(), msg.getTheta(), targetPoint.x,
							targetPoint.y, startTheta, -TARGET_THRESHOLD, TARGET_THRESHOLD)) {
						System.out.println("GRASPING: We are within range of target");
						// TBD
						//(new GUIPointMessage(tX, tY, MapGUI.X_POINT)).publish();
						startingMove = true;
						setVelocity(0.0, 0.0);
						//Robot.setVelocity(0.0, 0.0);
						fsmState = RoboFSM.COLLECTING; 
					}
					break;
				}

				case BLIND_APPROACH: {

					//System.out.println("GRASPING: MOVE FORWARD");
					// TODO
					//if (Math.abs(blobTrack.target
					// check distance to target and decrease standoff

					// if object is lost, go back to VSSEARCH

					//this is just a placeholder for moving forward.
					//System.out.println("GRASPING: *** MOVE_FORWARD *** " + startingMove);
					if(startingMove) {
						startPoint = new Point2D.Double();
						startPoint.x = msg.
						getX();
						startPoint.y = msg.getY();
						startTheta = msg.getTheta();
						targetPoint = new Point2D.Double();
						targetPoint.x = startPoint.x +
						moveDistance*Math.cos(startTheta);
						targetPoint.y = startPoint.y +
						moveDistance*Math.sin(startTheta);
						targetTheta = startTheta;
						startingMove = false;
					}

					if(moveTowardTarget(msg.getX(), msg.getY(), msg.getTheta(), targetPoint.x,
							targetPoint.y, startTheta, -TARGET_THRESHOLD, TARGET_THRESHOLD)) {
						System.out.println("GRASPING: We are within range of target");
						// TBD
						//(new GUIPointMessage(tX, tY, MapGUI.X_POINT)).publish();
						startingMove = true;
						setVelocity(0.0, 0.0);
						//					Robot.setVelocity(0.0, 0.0);
						fsmState = RoboFSM.ENGAGE_BLOCK; 
					}
					break;
				}

				case PULL_BACK: {
			
					System.out.println("GRASPING: *** MOVE_BACKWARD *** " + startingMove);
					if(startingMove) {
						startPoint = new Point2D.Double();
						startPoint.x = msg.getX();
						startPoint.y = msg.getY();
						startTheta = msg.getTheta();
						targetPoint = new Point2D.Double();
						targetPoint.x = startPoint.x -
						moveDistance*Math.cos(startTheta);
						targetPoint.y = startPoint.y -
						moveDistance*Math.sin(startTheta);
						targetTheta = startTheta;
						startingMove = false;
					}

					if(moveTowardTarget(msg.getX(), msg.getY(), msg.getTheta(), targetPoint.x,
							targetPoint.y, startTheta, -TARGET_THRESHOLD, 0.10)) {
						// TBD
						//(new GUIPointMessage(tX, tY, MapGUI.X_POINT)).publish();
						startingMove = true;
						setVelocity(0.0, 0.0);
						//					Robot.setVelocity(0.0, 0.0);
						fsmState = RoboFSM.RELEASE_BLOCK; 
					}
					break;
				}

				case MOVE_BACKWARD: {
			
					System.out.println("GRASPING: *** MOVE_BACKWARD *** " + startingMove);
					if(startingMove) {
						startPoint = new Point2D.Double();
						startPoint.x = msg.getX();
						startPoint.y = msg.getY();
						startTheta = msg.getTheta();
						targetPoint = new Point2D.Double();
						targetPoint.x = startPoint.x -
						moveDistance*Math.cos(startTheta);
						targetPoint.y = startPoint.y -
						moveDistance*Math.sin(startTheta);
						targetTheta = startTheta;
						startingMove = false;
					}

					if(moveTowardTarget(msg.getX(), msg.getY(), msg.getTheta(), targetPoint.x,
							targetPoint.y, startTheta, -TARGET_THRESHOLD, 0.10)) {
						// TBD
						//(new GUIPointMessage(tX, tY, MapGUI.X_POINT)).publish();
						startingMove = true;
						setVelocity(0.0, 0.0);
						//					Robot.setVelocity(0.0, 0.0);
						fsmState = RoboFSM.SET_ARM_TO_COLLECT; 
					}
					break;
				}
			}
		}
		else if (SERVO_MODE == ASSEMBLING) {
			switch (fsmState) {

				
				case SET_ARM_RETRACTED: {
					System.out.println("Retracting Arm ");
						break;
				}
				case ASSEMBLY_ROTATE: {
					System.out.println("Assembly done: Rotating now");

					if (!rotating) {
						startPoint = new Point2D.Double();
						startPoint.x = msg.getX();
						startPoint.y = msg.getY();
						startTheta = msg.getTheta();
						targetPoint = new Point2D.Double();
						targetPoint.x = startPoint.x -
						0.1*Math.cos(startTheta);
						targetPoint.y = startPoint.y -
						0.1*Math.sin(startTheta);
						targetTheta = startTheta;
						rotating = true;
						//
					}

					if(rotateTowardTarget(msg.getX(), msg.getY(), msg.getTheta(), targetPoint.x,
							targetPoint.y, DIR_FORWARD)) {
						// TBD
						//(new GUIPointMessage(tX, tY, MapGUI.X_POINT)).publish();
						rotating = false;
						setVelocity(0.0, 0.0);//					Robot.setVelocity(0.0, 0.0);
						fsmState = RoboFSM.OFF;
						setGrasping(OFF, true, true);
          }
					break; 

				}
				case MOVE_BACKWARD: {
			
					System.out.println("GRASPING: *** MOVE_BACKWARD *** " + startingMove);
					if(startingMove) {
						startPoint = new Point2D.Double();
						startPoint.x = msg.getX();
						startPoint.y = msg.getY();
						startTheta = msg.getTheta();
						targetPoint = new Point2D.Double();
						targetPoint.x = startPoint.x -
						BACK_DISTANCE*Math.cos(startTheta);
						targetPoint.y = startPoint.y -
						BACK_DISTANCE*Math.sin(startTheta);
						targetTheta = startTheta;
						startingMove = false;
						
					}
					
					if(moveTowardTarget(msg.getX(), msg.getY(), msg.getTheta(), targetPoint.x,
							targetPoint.y, startTheta, -TARGET_THRESHOLD, 0.10)) {
						// TBD
						//(new GUIPointMessage(tX, tY, MapGUI.X_POINT)).publish();
						startingMove = true;
						fsmState=RoboFSM.ASSEMBLY_ROTATE;
						setVelocity(0.0, 0.0);//					Robot.setVelocity(0.0, 0.0);
					}
					break;
				}
			}
		}
	}

	public void setVelocity(double rotVel, double transVel) {
    /*
    if (Math.abs(rotVel-prevRotVel) < 1e-5 &&
        Math.abs(transVel-prevTransVel) < 1e-4) {
      return;
    }
    */
    prevRotVel = rotVel;
    prevTransVel = transVel;
		MotionMsg motionMsg = motionPub.newMessage();
		motionMsg.setRotationalVelocity(rotVel);
		motionMsg.setTranslationalVelocity(transVel);
		motionPub.publish(motionMsg);
	}

	public void setGrasping(int graspingMode, boolean collected, boolean found){
		GraspingMsg graspingMsg = graspingPub.newMessage();
		graspingMsg.setServomode(graspingMode);
		graspingMsg.setCollected(collected);
		//graspingMsg.setAssembled(assembled);
		graspingMsg.setFound(found);
		graspingPub.publish(graspingMsg);
	}



	/**
	 * <p>Moves from current (x, y, heading) towards target point (tX, tY) in
	 * a forward (1) or backward (-1) direction.<\p>
	 */
	private boolean rotateTowardTarget(double x, double y, double heading,
			double tX, double tY, int direction) {
		System.out.println("GRASPING:   - current: x:" + x + " y:" + y);
		System.out.println("GRASPING:   - target: x:" + tX + " y:" + tY);

		// distance to target
		double tD = Math.hypot((x-tX), (y-tY));
		//System.out.println("GRASPING:   Distance to target: " + tD);

		if (direction == DIR_BACKWARD) {
			heading = heading - Math.PI;
		}

		//cosine and sine of actual heading
		double cActual = Math.cos(heading);
		double sActual = Math.sin(heading);

		//cosine and sine of desired heading
		double cDesired = (tX-x)/tD;
		double sDesired = (tY-y)/tD;

		//cosine and sine of error angle
		double cError = cDesired*cActual+sDesired*sActual;
		double sError = sDesired*cActual-cDesired*sActual;

		double thetaError = Math.atan2(sError, cError);

		if (Math.abs(thetaError) < THETA_THRESHOLD){
			return true;
		}
		else {
			double rv = WHEEL_RV_GAIN * thetaError;
			if (rv > WHEEL_MAX_RV) {
				rv = WHEEL_MAX_RV;
			}
			if (rv < -WHEEL_MAX_RV) {
				rv = -WHEEL_MAX_RV;
			}

			System.out.println("GRASPING:  thetaError: " + thetaError + " rv: " + rv
          + " transDistance: " + tD);

			setVelocity(rv, 0);
				
			return false;
		}
	}

	/**
	 * <p>Moves from current (x, y, heading) towards target point (tX, tY) in
	 * a forward (1) or backward (-1) direction.<\p>
	 */
	private boolean moveTowardTarget(double x, double y, double heading,
			double tX, double tY, double desiredHeading,
      double tDthreshLeft, double tDthreshRight) {
		System.out.println("GRASPING:   - current: x:" + x + " y:" + y);
		System.out.println("GRASPING:   - target: x:" + tX + " y:" + tY);

    //cosine and sine of actual heading
    //cosine and sine of desired heading
    double cActual = Math.cos(heading);
    double sActual = Math.sin(heading);
		// displacement to target
    double transDisplacement = cActual*(tX-x)+sActual*(tY-y);

    if (tDthreshLeft < transDisplacement &&
        transDisplacement < tDthreshRight) {
			return true;
		} else {
      double thetaError = (desiredHeading - heading)%(2.0*Math.PI);
      if (thetaError > Math.PI) {
        thetaError -= 2.0*Math.PI;
      }
			double rv = WHEEL_RV_GAIN * thetaError * 0.1;
			if (rv > WHEEL_MAX_RV) {
				rv = WHEEL_MAX_RV;
			}
			if (rv < -WHEEL_MAX_RV) {
				rv = -WHEEL_MAX_RV;
			}

      double tvBias = -(tDthreshLeft + tDthreshRight)/2.0;
			double tv = (tvBias+transDisplacement) * WHEEL_TV * 2.0;
      if (tv > WHEEL_TV) {
        tv = WHEEL_TV;
      }
      if (tv < -WHEEL_TV) {
        tv = -WHEEL_TV;
      }
			
			System.out.println("GRASPING:  thetaError: " + thetaError + " rv: " + rv
          + " tv: " + tv + " transDistance: " + transDisplacement);

			setVelocity(rv, tv);
		}
		return false;
	}


	private double target_red_hue_level = 0.00;
	private double target_blue_hue_level = 0.63; 
	private double target_yellow_hue_level = 0.167;
	private double target_green_hue_level = 0.4; 

	public double target_red_sat_level=0.5; // (Solution)
	public double target_blue_sat_level=0.35; // (Solution)
	public double target_yellow_sat_level=0.5; // (Solution)
	public double target_green_sat_level=0.5; // (Solution)

	private double hue_threshold= 0.05;
	private double blob_size_threshold = 0.015;
	private double target_radius = 0.1;
	private double desired_fixation_distance = .5;
	private double translation_error_tolerance = .005;
	private double translation_velocity_gain = 0.5;
	private double translation_velocity_max = .10;
	private double rotation_error_tolerance = Math.PI/180.0*0.3;
	private double rotation_velocity_gain = 0.4;
	private double rotation_velocity_max = 0.10;
	private boolean use_gaussian_blur = false;//true;
  int videoCounter = 0;


  private double desired_fixation_distance_approach = .31;
  private double rotation_velocity_gain_approach = 0.3;
  private double rotation_velocity_max_approach = 0.05;
  private double translation_error_tolerance_approach = .005;
  private double translation_velocity_gain_approach = 0.2;
	private double translation_velocity_max_approach = .04;

	/**
	 * <p>Handle an image message. Perform blob tracking and
	 * servo robot towards target.</p>
	 *
	 * @param a received camera message
	 */
	public synchronized void handle(byte[] rawImage, int width, int height) {

    ++videoCounter;
    if (videoCounter % 10 != 0) {
      return;
    }
		// on first camera message, create new BlobTracking instance
		if ( blobTrack == null) {
			System.out.println("GRASPING: Blobtracking");
			blobTrack = new BlobTracking(width, height, freeCellClient);
      blobTrack.updateLocation(curLocX, curLocY, curLocTheta);

      /*
			blobTrack.targetRedHueLevel = target_red_hue_level;
			blobTrack.targetBlueHueLevel = target_blue_hue_level;
			blobTrack.targetYellowHueLevel = target_yellow_hue_level;
			blobTrack.targetGreenHueLevel = target_green_hue_level;

			blobTrack.redSaturationLevel = target_red_sat_level;
			blobTrack.blueSaturationLevel = target_blue_sat_level;
			blobTrack.yellowSaturationLevel = target_yellow_sat_level;
			blobTrack.greenSaturationLevel = target_green_sat_level;
      */


			//blobTrack.hueThreshold = hue_threshold;
			blobTrack.blobSizeThreshold = blob_size_threshold;
			blobTrack.targetRadius = target_radius;
			blobTrack.desiredFixationDistance = desired_fixation_distance;
			blobTrack.translationErrorTolerance = translation_error_tolerance;
			blobTrack.translationVelocityGain = translation_velocity_gain;
			blobTrack.translationVelocityMax = translation_velocity_max;
			blobTrack.rotationErrorTolerance = rotation_error_tolerance;
			blobTrack.rotationVelocityGain = rotation_velocity_gain;
			blobTrack.rotationVelocityMax = rotation_velocity_max;
			blobTrack.useGaussianBlur = use_gaussian_blur;
			System.out.println("Initializing blotTrack: done");
		}

    Image src = new Image(rawImage, width, height);
    Image dest = new Image(rawImage, width, height);
    blobTrack.apply(src, dest);

    sensor_msgs.Image pubImage = vidPub.newMessage();
    pubImage.setWidth(width);
    pubImage.setHeight(height);
    pubImage.setEncoding("rgb8");
    pubImage.setIsBigendian((byte)0);
    pubImage.setStep(width*3);
    pubImage.setData(org.jboss.netty.buffer.ChannelBuffers.
                      copiedBuffer(org.jboss.netty.buffer.ChannelBuffers.LITTLE_ENDIAN, dest.toArray()));
    vidPub.publish(pubImage);

		// to calibrate standoff distance
		// wait until breakbeam is tripped, then output that range
		// perform multiple trials

		// 2 phase approach
		// Mode 1: VISUAL_SERVO_SEARCH
		//   - go until we are .5 m from brick (Camera Handler)
		//   - change state to SET_ARM_FOR_GRASP
		//   - change state to VISUAL_SERVO_GRASP
		// Mode 2: VISUAL_SERVO_APPROACH
		//   - keep decrementing blobTrack.targetDistance until breakbeam tripped
		//      - camera handler - decrement standoff on each iteration, with
		//        smaller and smaller sizes as we go
		//      - gripper - when breakbeam is tripped, proceed as normal

		// TODO: if we lose target during SEARCH, nothing
		// TODO: if we lose target during APPROACH, go back to search

    if (fsmState==RoboFSM.VISUAL_SERVO_SEARCH){
      double curTime = CurTime();
      if (!blobTrack.targetDetected) {
        if (lastDetectionTime > -1e17 &&
            curTime - lastDetectionTime > VS_TIMEOUT_SEC) {
          System.out.println("Timed out with the detection");
          lastDetectionTime = -1e18;
          fsmState = RoboFSM.OFF;
          setGrasping(OFF, false, false);
          return;
        }
      } else {
        lastDetectionTime = curTime;
      }

      System.out.println("GRASPING: VISUAL SERVO SEARCH");
			System.out.println("GRASPING:   range, bearing:" +
          blobTrack.targetRange + ", " +
					(blobTrack.targetBearing*180.0/Math.PI));

			if (Math.abs(blobTrack.rotationVelocityCommand)<0.001 && Math.abs(blobTrack.targetRange-SEARCH_STANDOFF) < EPS_SEARCH_STANDOFF) {
				//fsmState = RoboFSM.SET_ARM_RETRACTED;<<

				if (SERVO_MODE == WALL_COLLECTING){
					fsmState = RoboFSM.SET_ARM_TO_PULL;
				} else {
					fsmState = RoboFSM.SET_ARM_TO_COLLECT;
				}
				setVelocity(0.0, 0.0);
			} else {
				// move robot towards target
				setVelocity(blobTrack.rotationVelocityCommand, blobTrack.translationVelocityCommand);
			}
		} else {
	
			if (blobTrack.targetDetected) {

				if (blobTrack.targetNotRisky){
					setGrasping(COLLECTING, true, true);
				} else {
					setGrasping(COLLECTING, false, true);
				}
				
			} 
		
		}
	}

	/**
	 * Joint Controller Class
	 */
	private class JointController {

		double thetaDesired;
		double thetaCommanded;
		double maxThetaChange = .1;
		double mostRecentPosition = 0;
		boolean mostRecentPositionValid = false;

		/* Each joint needs a mapping from radians to the rawPwm values that
		 * Carmen sends to the servo. These can be callibrated using the gui.
		 */

		// Characterize linear theta-PWM relation as y=mx+b, where y=theta,
		// m=thetaPerPwm, x=pwm, b=thetaIntercept
		double thetaPerPwm;
		double thetaIntercept;

		// store servo bounds
		double servoPwmMin;
		double servoPwmMax;
		double thetaMin;
		double thetaMax;
		double thetaSafe;

		boolean servoOn=true;

		int state;

		/**
		 * <p>Constructs a JointController object<\p>
		 */
		public JointController(double servoPwmMin, double servoPwmMax,
				double thetaMin, double thetaMax,
				double thetaPerPwm, double thetaIntercept,
				double maxThetaChange) {
			this.servoPwmMin = servoPwmMin;
			this.servoPwmMax = servoPwmMax;
			this.thetaMin = thetaMin;
			this.thetaMax = thetaMax;
			this.thetaPerPwm = thetaPerPwm;
			this.thetaIntercept = thetaIntercept;
			this.maxThetaChange = maxThetaChange;
			this.thetaSafe = (thetaMin+thetaMax)/2.0;
		}

		/**
		 * <p>Contructs a JointController object and calibrates based on the
		 * maximum and minumum (pwm, theta) measurements of the servo.<\p>
		 */
		public JointController(double servoPwmMin, double servoPwmMax,
				double thetaAtPwmMin, double thetaAtPwmMax,
				double maxThetaChange) {
			this.servoPwmMin = servoPwmMin;
			this.servoPwmMax = servoPwmMax;
			this.maxThetaChange = maxThetaChange;
			calibrate(servoPwmMin, thetaAtPwmMin, servoPwmMax, thetaAtPwmMax);
			thetaMin = Math.min(thetaAtPwmMin, thetaAtPwmMax);
			thetaMax = Math.max(thetaAtPwmMin, thetaAtPwmMax);
			this.thetaSafe = (thetaMin+thetaMax)/2.0;
		}

		/**
		 * <p>Calibrates the pwm-theta characterization for this servo based
		 * on two (pwm, theta) datapoints<\p>
		 */
		public void calibrate(double pwm1, double theta1, double pwm2,
				double theta2) {
			double delta_pwm = pwm2 - pwm1;
			double delta_theta = theta2 - theta1;

			thetaPerPwm = delta_theta / delta_pwm;
			thetaIntercept = theta1 - (thetaPerPwm*pwm1);
		}

		/**
		 * <p>Prints the state of this JointController<\p>
		 */
		public void printState() {
			System.err.println("  ControllerState: AtDesired: "+isAtDesired()+" "
					+thetaDesired);
		}

		/**
		 * <p>Sets the state of this JointController<\p>
		 */
		public void setState(int state) {
			this.state = state;
		}
    
    public int getState() {
      return this.state;
    }

		/**
		 * <p>Set the desired theta for this servo. Multiple behaviors may
		 * attempt to set desired</p>
		 */
		public void setDesired(double desired) {
			thetaDesired = desired;
		}

		/**
		 * <p>Determine if servo has reached its desired theta<\p>
		 */
		public boolean isAtDesired() {
			return thetaCommanded == thetaDesired;
		}

		/**
		 * <p>Enable (1) or disable (0) servo operation<\p>
		 */
		public void enableServo(boolean enable) {
			servoOn = enable;
		}

		/**
		 * <p>Convert PWM value to joint angle in radians<\p>
		 */
		public double pwmToTheta(double pwm) {
			if (pwm > servoPwmMax) {
				pwm = servoPwmMax;
			}
			if (pwm < servoPwmMin) {
				pwm = servoPwmMin;
			}
			return (thetaPerPwm*pwm) + thetaIntercept;
		}

		/**
		 * <p>Convert PWM value to joint angle in degrees<\p>
		 */
		public double pwmToDegrees(double pwm) {
			return 180.0*pwmToTheta(pwm)/Math.PI;
		}

		/**
		 * <p>Convert joint angle in radians to PWM value<\p>
		 */
		public double thetaToPwm(double theta) {
			if (theta > thetaMax) {
				theta = thetaMax;
			}
			if (theta < thetaMin) {
				theta = thetaMin;
			}
			return (theta-thetaIntercept) / thetaPerPwm;
		}

		/**
		 * <p>Control Step
		 * At the end of each control cycle, returns the value to go to carmen
		 * <\p>
		 */
		public long step(double desired) {
			thetaDesired = desired;

			if ( !mostRecentPositionValid ) {
				System.out.println("GRASPING:     no msg received, sending to safe position, cmd=" + thetaSafe);
				return (long)thetaToPwm(thetaSafe);
			}

			// ramp the commanded to the desired
			double limit = maxThetaChange; // max rads can change

			if ( thetaDesired > mostRecentPosition )
				thetaCommanded = Math.min(thetaDesired, mostRecentPosition+limit);
			else
				thetaCommanded = Math.max(thetaDesired, mostRecentPosition-limit);

			if (servoOn) {
        /*
				if ( Math.abs(mostRecentPosition-thetaDesired) > 0.01 )
					System.out.println("cur=" + mostRecentPosition + ", goal=" + thetaDesired + ", cmd=" + thetaCommanded);
        */
				return (long)thetaToPwm(thetaCommanded);
			} else
				return (long)thetaToPwm(thetaSafe);
		}

		public void update(double currentPwm) {
			this.mostRecentPosition = pwmToTheta(currentPwm);
			this.mostRecentPositionValid = true;
		}

		/**
		 * <p>Hold a desired pose<\p>
		 */
		public long step() {
			return step(thetaDesired);
		}

	}

	/**
	 *
	 */
	private class ShoulderController extends JointController {

		// SHOULDER STATES
		static final int SHOULDER_RETRACTED = 0;
		static final int SHOULDER_EXTENDED = 1;
		static final int SHOULDER_RETRACTING = 2;
		static final int SHOULDER_EXTENDING = 3;
    	static final int SHOULDER_INITIALIZE = 4;
  	  	static final int SHOULDER_FINALIZE = 4;

		/*
    static final double servoPwmMin = 5637;
    static final double servoPwmMax = 19379;
    static final double pwmAtThetaZero = 11627;
		 */
		static final double servoPwmMin = 500;
		static final double thetaAtPwmMin = -90*Math.PI/180;
		static final double servoPwmMax = 2200;
		static final double thetaAtPwmMax = 90*Math.PI/180;

		//static final double thetaPerPwmTick = (Math.PI) /
		//  (17265-pwmAtThetaZero); // calibration
		//static final double thetaPerPwmTick = (Math.PI) /
		//  (servoPwmMin - pwmAtThetaZero); // calibration

		static final double maxThetaChange = 0.03;

		//final double poseReleasing = thetaMax;    //radians
		final double poseRetracted = pwmToTheta(2200);                           //radians
		//final double poseGrasp =
		final double poseParallel = pwmToTheta(1450);

		final double poseCollecting = pwmToTheta(1500);

		final double poseGating = pwmToTheta(650);

		final double poseSetShoulderToPull = pwmToTheta(1100); 
		final double poseSetShoulderRelease = pwmToTheta(1500);   
		final double poseShoulderEngage = pwmToTheta(980);                   //radians

		public ShoulderController() {
			super(servoPwmMin, servoPwmMax, thetaAtPwmMin, thetaAtPwmMax,
					maxThetaChange);
		}

		@Override public void printState() {
			System.err.println("Shoulder: Extended: "+(state==SHOULDER_EXTENDED)
					+ " Retracted: "+(state==SHOULDER_RETRACTED) +
					state);
			super.printState();
		}

		public long step(ArmMsg msg) {
			if(SERVO_MODE == COLLECTING || SERVO_MODE == WALL_COLLECTING) {return step_COLLECTING(msg);}
			else if(SERVO_MODE == ASSEMBLING) {return step_ASSEMBLING(msg);}
			else {return (long) 0;}
		}

		public long step_COLLECTING(ArmMsg msg) {
			long returnVal;

			switch (fsmState) {

				case INITIALIZE_ARM: {
					if (wristControl.isAtDesired()){
						returnVal = super.step(poseGating);
						if (isAtDesired()) {
							System.out.println("GRASPING:   - Shoulder is initialized");
						}
					} else {
						returnVal = super.step(poseRetracted);
						System.out.println("GRASPING:   - Shoulder is waiting for wrist");
					}	
					
					break;
				}

				case SET_ARM_TO_COLLECT: {
					returnVal = super.step(poseCollecting);
					if(isAtDesired()) {
						System.out.println("GRASPING:   - Shoulder is at desired");
					}
					break;
				}

				case SET_ARM_TO_PULL: {
					returnVal = super.step(poseSetShoulderToPull);
					if(isAtDesired()) {
						System.out.println("GRASPING:   - Shoulder is at desired");
					}
					break;
				}

				case ENGAGE_BLOCK: {
					returnVal = super.step(poseShoulderEngage);
					if(isAtDesired()) {
						System.out.println("GRASPING:   - Shoulder is at desired");
					}
					break;
				}

				case RELEASE_BLOCK: {
					returnVal = super.step(poseSetShoulderRelease);
					if(isAtDesired()) {
						System.out.println("GRASPING:   - Shoulder is at desired");
					}
					break;
				}

				case SET_ARM_TO_GATE: {
					returnVal = super.step(poseGating);
					if(isAtDesired()) {
						System.out.println("GRASPING:   - Shoulder is at desired");
					}
					break;
				}

				case SET_ARM_RETRACTED: {
					returnVal = super.step(poseRetracted);
					if(isAtDesired()) {
						System.out.println("GRASPING:   - Shoulder is at desired");
					}
					break;
				}

				case COLLECTING: {
					System.out.println("GRASPING: *** COLLECTING OBJECT ***");
					returnVal = super.step(poseGating);
					break;
				}

				case RELEASING: {
					System.out.println("GRASPING: *** LOWER OBJECT ***");
					returnVal = super.step(poseRetracted);
					//setDesired(poseExtended);
					//returnVal = lowerShoulder(msg, returnVal);
					break;
				}

				default : {
					returnVal = super.step();
				}
			}

			return returnVal;
		}
		public long step_ASSEMBLING(ArmMsg msg) {
			long returnVal;

			switch (fsmState) {

				case INITIALIZE_ARM: {
					if (wristControl.isAtDesired()){
						returnVal = super.step(poseGating);
						if (isAtDesired()) {
							System.out.println("GRASPING:   - Shoulder is initialized");
						}
					} else {
						returnVal = super.step(poseRetracted);
						System.out.println("GRASPING:   - Shoulder is waiting for wrist");
					}	
					
					break;
				}

				case SET_ARM_TO_COLLECT: {
					returnVal = super.step(poseCollecting);
					if(isAtDesired()) {
						System.out.println("GRASPING:   - Shoulder is at desired");
					}
					break;
				}

				case SET_ARM_TO_PULL: {
					returnVal = super.step(poseSetShoulderToPull);
					if(isAtDesired()) {
						System.out.println("GRASPING:   - Shoulder is at desired");
					}
					break;
				}

				case ENGAGE_BLOCK: {
					returnVal = super.step(poseShoulderEngage);
					if(isAtDesired()) {
						System.out.println("GRASPING:   - Shoulder is at desired");
					}
					break;
				}

				case RELEASE_BLOCK: {
					returnVal = super.step(poseSetShoulderRelease);
					if(isAtDesired()) {
						System.out.println("GRASPING:   - Shoulder is at desired");
					}
					break;
				}

				case SET_ARM_TO_GATE: {
					returnVal = super.step(poseGating);
					if(isAtDesired()) {
						System.out.println("GRASPING:   - Shoulder is at desired");
					}
					break;
				}

				case SET_ARM_RETRACTED: {
					returnVal = super.step(poseRetracted);
					if(isAtDesired()) {
						System.out.println("GRASPING:   - Shoulder is at desired");
					}
					break;
				}

				case COLLECTING: {
					System.out.println("GRASPING: *** COLLECTING OBJECT ***");
					returnVal = super.step(poseGating);
					break;
				}

				case RELEASING: {
					System.out.println("GRASPING: *** LOWER OBJECT ***");
					returnVal = super.step(poseRetracted);
					//setDesired(poseExtended);
					//returnVal = lowerShoulder(msg, returnVal);
					break;
				}

				default : {
					returnVal = super.step();
				}
			}

			return returnVal;
		}


	}


	/**
	 *
	 * For now, functionally identical to ShoulderController, different
	 * calibration and tuning
	 */
	private class WristController extends JointController {

		static final int WRIST_RETRACTED = 0;
		static final int WRIST_EXTENDED = 1;
		static final int WRIST_RETRACTING = 2;
		static final int WRIST_EXTENDING = 3;
    	static final int WRIST_INITIALIZE = 4;

		static final double servoPwmMin = 450;
		static final double thetaAtPwmMin = 90.0*Math.PI/180;//-110*Math.PI/180;
		static final double servoPwmMax = 2500;
		static final double thetaAtPwmMax = -110.0*Math.PI/180;

		static final double maxThetaChange = .05;    //Rad/sec

		double poseExtended = pwmToTheta(800);//thetaMin;
		double poseCollecting = pwmToTheta(2400);//thetaMax;
		double poseSetWristToPull = pwmToTheta(800);
		double poseWristEngage = pwmToTheta(1100);
		double poseGating = pwmToTheta(2600);
		//double poseReleasing = thetaMin;  //radians

		public WristController() {
			super(servoPwmMin, servoPwmMax, thetaAtPwmMin, thetaAtPwmMax,
					maxThetaChange);
		}

		@Override public void printState() {
			System.err.println("Wrist: Extended: "+(state == WRIST_EXTENDED)
					+ " Retracted: "+(state == WRIST_RETRACTED) +
					state);
			super.printState();
		}

		public long step(ArmMsg msg) {
			if(SERVO_MODE == COLLECTING || SERVO_MODE == WALL_COLLECTING) {return step_COLLECTING(msg);}
			else if(SERVO_MODE == ASSEMBLING) {return step_ASSEMBLING(msg);}
			else {return (long) 0;}
		}


		public long step_COLLECTING(ArmMsg msg) {
			long returnVal;

			switch (fsmState) {

				case INITIALIZE_ARM: {
					returnVal = super.step(poseGating);
					if (isAtDesired()) {
						System.out.println("GRASPING:   - Wrist is initialized");
					}
					break;
				}

				case SET_ARM_TO_COLLECT: {
					returnVal = super.step(poseCollecting);

					if(isAtDesired()) {
						System.out.println("GRASPING:   - Wrist is ready to collect");
					}

					break;
				}

				case SET_ARM_TO_GATE: {
					returnVal = super.step(poseGating);

					if(isAtDesired()) {
						System.out.println("GRASPING:   - Wrist is in gating position");
					}

					break;
				}



				case SET_ARM_TO_PULL: {
					returnVal = super.step(poseSetWristToPull);
					if(isAtDesired()) {
						System.out.println("GRASPING:   - Shoulder is at desired");
					}
					break;
				}

				case ENGAGE_BLOCK: {
					returnVal = super.step(poseWristEngage);
					if(isAtDesired()) {
						System.out.println("GRASPING:   - Shoulder is at desired");
					}
					break;
				}

				case RELEASE_BLOCK: {
					returnVal = super.step(poseSetWristToPull);
					if(isAtDesired()) {
						System.out.println("GRASPING:   - Shoulder is at desired");
					}
					break;
				}

				case COLLECTING: {
					returnVal = super.step(poseGating);
					if(isAtDesired()) {
						System.out.println("GRASPING:   - Wrist is in gating position");
					}

					break;
				}

				case RELEASING: {
					returnVal = super.step(poseGating);
					if(isAtDesired()) {
						System.out.println("GRASPING:   - Wrist is in gating position");
					}

					break;
				}

				default: {
					returnVal = super.step();
					break;
				}
			}

			return returnVal;
		}

		public long step_ASSEMBLING(ArmMsg msg) {
			long returnVal;

			switch (fsmState) {

				case INITIALIZE_ARM: {
					returnVal = super.step(poseGating);
					if (isAtDesired()) {
						System.out.println("GRASPING:   - Wrist is initialized");
					}
					break;
				}

				case SET_ARM_TO_COLLECT: {
					returnVal = super.step(poseCollecting);

					if(isAtDesired()) {
						System.out.println("GRASPING:   - Wrist is ready to collect");
					}

					break;
				}

				case SET_ARM_TO_GATE: {
					returnVal = super.step(poseGating);

					if(isAtDesired()) {
						System.out.println("GRASPING:   - Wrist is in gating position");
					}

					break;
				}



				case SET_ARM_TO_PULL: {
					returnVal = super.step(poseSetWristToPull);
					if(isAtDesired()) {
						System.out.println("GRASPING:   - Shoulder is at desired");
					}
					break;
				}

				case ENGAGE_BLOCK: {
					returnVal = super.step(poseWristEngage);
					if(isAtDesired()) {
						System.out.println("GRASPING:   - Shoulder is at desired");
					}
					break;
				}

				case RELEASE_BLOCK: {
					returnVal = super.step(poseSetWristToPull);
					if(isAtDesired()) {
						System.out.println("GRASPING:   - Shoulder is at desired");
					}
					break;
				}

				case COLLECTING: {
					returnVal = super.step(poseGating);
					if(isAtDesired()) {
						System.out.println("GRASPING:   - Wrist is in gating position");
					}

					break;
				}

				case RELEASING: {
					returnVal = super.step(poseGating);
					if(isAtDesired()) {
						System.out.println("GRASPING:   - Wrist is in gating position");
					}

					break;
				}

				default: {
					returnVal = super.step();
					break;
				}
			}

			return returnVal;
		}
	}


	/**
	 * <p>Given joint angles, find (x,y,theta) position of
	 * end effector<\p>
	 */
	public void forwardKinematics(double theta1, double theta2) {
    double L = 0.239, l = 0.135;
		System.out.println("GRASPING: Compute Forward Kinematics for:");
		System.out.println("GRASPING:    l1:" + L + " theta1:" + theta1);
		System.out.println("GRASPING:    l2:" + l + " theta2:" + theta2);

		double theta = theta1 + theta2;
		double x = L*Math.cos(theta1) + l*Math.cos(theta);
		double z = L*Math.sin(theta1) + l*Math.sin(theta);

		System.out.println("----------------------------------------");
		System.out.println("GRASPING:    X:" + x + " Y:" + z + " Theta:" + theta);
	}


	/**
	 * <p>Given end effector position, find joint angles<\p>
	 */
	public void inverseKinematics(double x, double z) {
		System.out.println("GRASPING: Compute Inverse Kinematics for:");
		System.out.println("GRASPING:    x:" + x + " z:" + z);
    double L = 0.239, l = 0.135;
    double gripperHeight = 0.06;
    double zOffset = 0.265, xOffset = 0.1;

    // Compute relative to parallel position of shoulder.
    x = x-xOffset;
    z = z-zOffset+gripperHeight/2.0;

    double coswrist = (x*x+z*z-L*L-l*l)/(2.0*L*l);
    double sinwrist = Math.sqrt(1.0-coswrist*coswrist);
    double thwrist = Math.acos(coswrist);
    double thshould = Math.atan2(z*(L+coswrist*l)-x*sinwrist*l,
                                 x*(L+coswrist*l)+z*sinwrist*l);

    // TODO: Move to specified position.
	}

  public void setBumpPressed(boolean value) {
  	//System.out.println("GRASPING: ////Bump Status////");
  	//System.out.println(bumpPressed);
  	//System.out.println("GRASPING: ////End Bump Status////");
    this.bumpPressed = value;
  }

  public double CurTime() {
    return System.currentTimeMillis()/1000.0;
  }

}
