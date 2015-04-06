package Grasping;

import java.awt.geom.Point2D;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.ArmMsg;
import org.ros.message.rss_msgs.BumpMsg;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import VisualServo.BlobTracking;
import VisualServo.Image;

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
  BLIND_APPROACH,
  DO_NOTHING,
	/**
	 * <p>FSM state: arm initial position.  The initial position is expected in
	 * fully retracted state, shoulder back, arm back, gripper closed.<\p>
	 */
	INITIALIZE_ARM,
	/**
	 * <p>FSM state: positioning arm for a grasp<\p>
	 */
	SET_ARM_FOR_GRASP,
	/**
	 * <p>FSM state: detect break-beam obstruction<\p>
	 */
	WAIT_FOR_BREAKBEAM,
	/**
	 * <p>FSM state: grasp object with gripper<\p>
	 */
	GRASP_OBJECT,
	/**
	 * <p>FSM state: lift object with shoulder<\p>
	 */
	LIFT_OBJECT,
	/**
	 * <p>FSM state: object has been dropped<\p>
	 */
	DROPPED_OBJECT,

	/**
	 * <p>FSM state: move forward by set transport distance<\p>
	 */
	MOVE_FORWARD,
	/**
	 * <p>FSM state: lower object with shoulder<\p>
	 */
	LOWER_OBJECT,
	/**
	 * <p>FSM state: release object from gripper<\p>
	 */
	RELEASE_OBJECT,
	/**
	 * <p>FSM state: translate backward to starting point<\p>
	 */
	MOVE_BACKWARD,
	VISUAL_SERVO_SEARCH,
	SET_ARM_FOR_APPROACH,
	VISUAL_SERVO_APPROACH,
  // Part 3A
  GRIPPER,  
}

public class Grasping implements NodeMain {
	//subscribes to rss/ArmStatus, rss/DigitalIO, rss/Odometry

	/**
	 * <p>Used to cleanly separate code fragments from different Parts of the
	 * lab<\p>
	 */
	public int LAB_PART;
	static final int PART_2A = 0;
	static final int PART_2B = 1;
	static final int PART_3A = 2;
	static final int PART_3B = 3;

	/**
	 * <p>Shoulder joint array index.  Note, current sense is on Orc port 0,
	 * which we want for shoulder.<\p>
	 */
	static final int SHOULDER_INDEX = 0;

	/**
	 * <p>Gripper joint array index.  Note, current sense is on Orc port 2,
	 * which we want for gripper.<\p>
	 */
	static final int GRIPPER_INDEX = 2;

	/**
	 * <p>Wrist joint array index<\p>
	 */
	static final int WRIST_INDEX = 1;

	/**
	 * <p>Shoulder controller instance<\p>
	 */
	ShoulderController shoulderControl = new ShoulderController();

	/**
	 * <p>Gripper controller instance<\p>
	 */
	GripperController gripperControl = new GripperController();

	/**
	 * <p>Wrist controller instance<\p>
	 */
	WristController wristControl = new WristController();

	/**
	 * <p>Entire Arm controller, array of JointControllers<\p>
	 */
	JointController[] armControl = {shoulderControl, wristControl, gripperControl};

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

	static final double EPS_SEARCH_STANDOFF = 0.05; // m
	static final double SEARCH_STANDOFF = 0.7;

	/**
	 * <p>Indicates first iteration through position controller<\p>
	 */
	boolean startingMove = true;

	/**
	 * <p>Starting pose of robot before moving<\p>
	 */
	Point2D.Double startPoint;

	/**
	 * <p>Target pose for robot at end of motion<\p>
	 */
	Point2D.Double targetPoint;

	/**
	 * <p>Distance to transport object (m)<\p>
	 */
	static final double TRANSPORT_DISTANCE = 0.5;

	/**
	 * <p>Translational velocity while moving (m/s).</p>
	 **/
	public static final double WHEEL_TV = 0.2;

	/**
	 * <p>Proportional gain for rotation controller while moving.</p>
	 **/
	public static final double WHEEL_RV_GAIN = 0.8;

	/**
	 * <p>Max rotational velocity while moving (rad/s).</p>
	 **/
	public static final double WHEEL_MAX_RV = 0.5;

	/**
	 * <p>Target reached threshold (m).</p>
	 **/
	public static final double TARGET_THRESHOLD = 0.1;

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
	private Publisher<org.ros.message.sensor_msgs.Image> vidPub;
	private double startTheta;
	private double targetTheta;
	private boolean bumpPressed;

	/**
	 * <p>Constructor for Grasping object<\p>
	 */
	public Grasping () {

		LAB_PART = PART_3A;//PART_2A;
		System.out.println("Lab part 3B"); // TODO

		switch (LAB_PART) {
			case PART_2A: {
        		fsmState = RoboFSM.INITIALIZE_ARM;
				gripperControl.setState(GripperController.GRIPPER_INITIALIZE);
				wristControl.setState(WristController.WRIST_INITIALIZE);
				shoulderControl.setState(ShoulderController.SHOULDER_INITIALIZE);
				break;
			}
      case PART_3A: {
        fsmState = RoboFSM.INITIALIZE_ARM;
        break;
      }
			case PART_3B: {
				fsmState = RoboFSM.INITIALIZE_ARM;
				break;
			}
			default:
				break;
		}

		System.out.println("Grasping class initialized");
	}

	private Subscriber<ArmMsg> armSub;
	private Subscriber<BumpMsg> bumpSub;
	private Subscriber<OdometryMsg> odoSub;
	private Subscriber<org.ros.message.sensor_msgs.Image> vidSub;

	@Override
	public void onStart(Node node){
		armPub = node.newPublisher("command/Arm", "rss_msgs/ArmMsg");
		motionPub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
		vidPub = node.newPublisher("/rss/blobVideo", "sensor_msgs/Image");

		armSub = node.newSubscriber("rss/ArmStatus", "rss_msgs/ArmMsg");
		armSub.addMessageListener(new ArmListener(this));
		bumpSub = node.newSubscriber("rss/BumpSensors", "rss_msgs/BumpMsg");
		bumpSub.addMessageListener(new BumpListener(this));
		odoSub = node.newSubscriber("rss/odometry", "rss_msgs/OdometryMsg");
		odoSub.addMessageListener(new OdometryListener(this));

		vidSub = node.newSubscriber("/rss/video", "sensor_msgs/Image");
		vidSub.addMessageListener(new MessageListener<org.ros.message.sensor_msgs.Image>() {
			@Override
			public void onNewMessage(org.ros.message.sensor_msgs.Image message) {
				byte[] rgbData = Image.RGB2BGR(message.data,  (int)message.width, (int)message.height);
				handle(rgbData, (int)message.width, (int)message.height);
			}
		});
	}

	@Override
	public void onShutdown(Node node){

	}

	@Override public void onShutdownComplete(Node node) {
	}

	@Override public GraphName getDefaultNodeName() {
		return new GraphName("rss/grasping");
	}

	/**
	 * <p>Handle an ArmMessage<\p>
	 */
  int counter = 0;
	public void handle(ArmMsg msg) {
 		gripperControl.update(msg.pwms[GRIPPER_INDEX]);
		wristControl.update(msg.pwms[WRIST_INDEX]);
		shoulderControl.update(msg.pwms[SHOULDER_INDEX]);

    ++counter;
    if (counter % 1 != 0) {
      return;
    }

    armTheta[GRIPPER_INDEX] = gripperControl.step(msg);
		armTheta[WRIST_INDEX] = wristControl.step(msg);
		armTheta[SHOULDER_INDEX] = shoulderControl.step(msg);

  	// Send arm pose to the Orc
		ArmMsg armMsg = new ArmMsg();
		armMsg.pwms = armTheta;
		armPub.publish(armMsg);

		// Part 2A - Arm gymnastics
    if (LAB_PART == PART_2A) {
      //System.out.println("in part 2A, handle message. Current state: " + fsmState);
      /*gripperControl.setState(GripperController.GRIPPER_OPENING);
      wristControl.setState(WristController.WRIST_EXTENDING);
      shoulderControl.setState(ShoulderController.SHOULDER_EXTENDING);*/
      switch (fsmState) {
        case DO_NOTHING: {
        	shoulderControl.setState(ShoulderController.SHOULDER_FINALIZE);
        }
        
        case INITIALIZE_ARM: {
					if ( gripperControl.isAtDesired()
							&& wristControl.isAtDesired()
							&& shoulderControl.isAtDesired() ) {
            System.out.println("Changing state to GRIPPER in part 2A");
            fsmState = RoboFSM.GRIPPER;
            gripperControl.setState(GripperController.GRIPPER_OPENING);
          }
          break;
        }
        
        case GRIPPER: {
          if (gripperControl.getState() == GripperController.GRIPPER_CLOSED) {
            fsmState = RoboFSM.DO_NOTHING;
          } else {
            System.out.println("Commanding gripper to pwm: " + armTheta[GRIPPER_INDEX] +
                                ", current pwm: " + msg.pwms[GRIPPER_INDEX]);
            //gripperControl.setState(GripperController.GRIPPER_OPENING);
            //armTheta[GRIPPER_INDEX] = gripperControl.step(msg);
          }
        }
      }
    } //if part 2A

    if (LAB_PART == PART_3A) {

			switch (fsmState) {

				case DO_NOTHING: {
					return;
				}

				case INITIALIZE_ARM: {
					if ( gripperControl.isAtDesired()
							&& wristControl.isAtDesired()
							&& shoulderControl.isAtDesired() ) {
						System.out.println("========================================================");
						System.out.println("Arm is now initialized (in retracted state)");
						// part 3b
						//fsmState = RoboFSM.SET_ARM_FOR_GRASP;
						// Part 4:
						fsmState = RoboFSM.SET_ARM_FOR_GRASP;
					}
					break;
				}

				case SET_ARM_FOR_GRASP: {
					System.out.println("SET_ARM_FOR_GRASP");
					if (gripperControl.isAtDesired() && wristControl.isAtDesired() &&
							shoulderControl.isAtDesired()) {
						System.out.println("Arm is SET FOR GRASP");
						// part 3b
						//fsmState = RoboFSM.BLIND_APPROACH;
						// part 4:
						fsmState = RoboFSM.WAIT_FOR_BREAKBEAM;

					}
					break;
				}

				case SET_ARM_FOR_APPROACH: {
					System.out.println("SET_ARM_FOR_APPROACH");
					if (gripperControl.isAtDesired() && wristControl.isAtDesired() &&
							shoulderControl.isAtDesired()) {
						System.out.println("  - Arm is SET FOR APPROACH");
						fsmState = RoboFSM.BLIND_APPROACH;
					}
					break;
				}

				case LOWER_OBJECT: {
					System.out.println("LOWER OBJECT");
					if (wristControl.isAtDesired() && shoulderControl.isAtDesired()) {
						System.out.println("Arm is LOWERED");
						fsmState = RoboFSM.RELEASE_OBJECT;
					}
				}
			}

		} // if Part 3b	
    
		if (LAB_PART == PART_3B) {

			switch (fsmState) {

				case DO_NOTHING: {
					return;
				}

				case INITIALIZE_ARM: {
					if ( gripperControl.isAtDesired()
							&& wristControl.isAtDesired()
							&& shoulderControl.isAtDesired() ) {
						System.out.println("========================================================");
						System.out.println("Arm is now initialized (in retracted state)");
						// part 3b
						//fsmState = RoboFSM.SET_ARM_FOR_GRASP;
						// Part 4:
						fsmState = RoboFSM.VISUAL_SERVO_SEARCH;
					}
					break;
				}

				case SET_ARM_FOR_GRASP: {
					System.out.println("SET_ARM_FOR_GRASP");
					if (gripperControl.isAtDesired() && wristControl.isAtDesired() &&
							shoulderControl.isAtDesired()) {
						System.out.println("Arm is SET FOR GRASP");
						// part 3b
						//fsmState = RoboFSM.BLIND_APPROACH;
						// part 4:
						fsmState = RoboFSM.VISUAL_SERVO_APPROACH;

					}
					break;
				}

				case SET_ARM_FOR_APPROACH: {
					System.out.println("SET_ARM_FOR_APPROACH");
					if (gripperControl.isAtDesired() && wristControl.isAtDesired() &&
							shoulderControl.isAtDesired()) {
						System.out.println("  - Arm is SET FOR APPROACH");
						fsmState = RoboFSM.BLIND_APPROACH;
					}
					break;
				}

				case LOWER_OBJECT: {
					System.out.println("LOWER OBJECT");
					if (wristControl.isAtDesired() && shoulderControl.isAtDesired()) {
						System.out.println("Arm is LOWERED");
						fsmState = RoboFSM.RELEASE_OBJECT;
					}
				}
			}

		} // if Part 3b	
  }


	/**
	 * <p>Handle an OdometryMessage<\p>
	 */
	public void handle(OdometryMsg msg) {
		//System.out.println("Odometry Message: (" + msg.x + " ," + msg.y + ", " + msg.theta + ")");

		// Part 3B - Object Transport FSM
		if (LAB_PART == PART_3B) {
			switch (fsmState) {

				case DO_NOTHING: {
					setVelocity(0.0, 0.0);
					break;
				}

				case INITIALIZE_ARM: {
					setVelocity(0.0, 0.0);
					break;
				}

				case MOVE_FORWARD: {
					System.out.println("*** MOVE_FORWARD *** " + startingMove);
					if(startingMove) {
						startPoint = new Point2D.Double();
						startPoint.x = msg.x;
						startPoint.y = msg.y;
						startTheta = msg.theta;
						targetPoint = new Point2D.Double();
						targetPoint.x = startPoint.x +
						TRANSPORT_DISTANCE*Math.cos(startTheta);
						targetPoint.y = startPoint.y +
						TRANSPORT_DISTANCE*Math.sin(startTheta);
						targetTheta = startTheta;
						startingMove = false;
					}

					if(moveTowardTarget(msg.x, msg.y, msg.theta, targetPoint.x,
							targetPoint.y, DIR_FORWARD)) {
						System.out.println("We are within range of target");
						// TBD
						//(new GUIPointMessage(tX, tY, MapGUI.X_POINT)).publish();
						startingMove = true;
						setVelocity(0.0, 0.0);
						//					Robot.setVelocity(0.0, 0.0);
						fsmState = RoboFSM.LOWER_OBJECT;
					}
					break;
				}

				case MOVE_BACKWARD: {
					System.out.println("*** MOVE_BACKWARD *** " + startingMove);
					if(startingMove) {
						startPoint = new Point2D.Double();
						startPoint.x = msg.x;
						startPoint.y = msg.y;
						startTheta = msg.theta;
						targetPoint = new Point2D.Double();
						targetPoint.x = startPoint.x -
						TRANSPORT_DISTANCE*Math.cos(startTheta);
						targetPoint.y = startPoint.y -
						TRANSPORT_DISTANCE*Math.sin(startTheta);
						targetTheta = startTheta;
						startingMove = false;
					}

					if(moveTowardTarget(msg.x, msg.y, msg.theta, targetPoint.x,
							targetPoint.y, DIR_BACKWARD)) {
						System.out.println("We are within range of target");

						startingMove = true;
						setVelocity(0.0, 0.0);
						System.out.println("Completed Object Transport");
						fsmState = RoboFSM.SET_ARM_FOR_GRASP;
					}
					break;
				}

				case BLIND_APPROACH:
					System.out.println("odom_handler:\tBumpPressed:\t"+bumpPressed);
					if(bumpPressed){
						setVelocity(0,0);
						fsmState = RoboFSM.GRASP_OBJECT;
					}
					else{
						setVelocity(0, BLIND_FORWARD_VELOCITY);
					}
					break;

					//    case STOP:
				case DROPPED_OBJECT:
					startingMove = true;
					fsmState = RoboFSM.MOVE_BACKWARD;
					//default:
					setVelocity(0.0, 0.0);
			}
		}
		// Part 3B - Object Transport FSM
		if (LAB_PART == PART_3A) {
			switch (fsmState) {

				case DO_NOTHING: {
					setVelocity(0.0, 0.0);
					break;
				}

				case INITIALIZE_ARM: {
					setVelocity(0.0, 0.0);
					break;
				}

				case MOVE_FORWARD: {
					System.out.println("*** MOVE_FORWARD *** " + startingMove);
					if(startingMove) {
						startPoint = new Point2D.Double();
						startPoint.x = msg.x;
						startPoint.y = msg.y;
						startTheta = msg.theta;
						targetPoint = new Point2D.Double();
						targetPoint.x = startPoint.x +
						TRANSPORT_DISTANCE*Math.cos(startTheta);
						targetPoint.y = startPoint.y +
						TRANSPORT_DISTANCE*Math.sin(startTheta);
						targetTheta = startTheta;
						startingMove = false;
					}

					if(moveTowardTarget(msg.x, msg.y, msg.theta, targetPoint.x,
							targetPoint.y, DIR_FORWARD)) {
						System.out.println("We are within range of target");
						// TBD
						//(new GUIPointMessage(tX, tY, MapGUI.X_POINT)).publish();
						startingMove = true;
						setVelocity(0.0, 0.0);
						//					Robot.setVelocity(0.0, 0.0);
						fsmState = RoboFSM.LOWER_OBJECT;
					}
					break;
				}

				case MOVE_BACKWARD: {
					System.out.println("*** MOVE_BACKWARD *** " + startingMove);
					if(startingMove) {
						startPoint = new Point2D.Double();
						startPoint.x = msg.x;
						startPoint.y = msg.y;
						startTheta = msg.theta;
						targetPoint = new Point2D.Double();
						targetPoint.x = startPoint.x -
						TRANSPORT_DISTANCE*Math.cos(startTheta);
						targetPoint.y = startPoint.y -
						TRANSPORT_DISTANCE*Math.sin(startTheta);
						targetTheta = startTheta;
						startingMove = false;
					}

					if(moveTowardTarget(msg.x, msg.y, msg.theta, targetPoint.x,
							targetPoint.y, DIR_BACKWARD)) {
						System.out.println("We are within range of target");

						startingMove = true;
						setVelocity(0.0, 0.0);
						System.out.println("Completed Object Transport");
						fsmState = RoboFSM.SET_ARM_FOR_GRASP;
					}
					break;
				}

				case BLIND_APPROACH:
					System.out.println("odom_handler:\tBumpPressed:\t"+bumpPressed);
					if(bumpPressed){
						setVelocity(0,0);
						fsmState = RoboFSM.GRASP_OBJECT;
					}
					else{
						setVelocity(0, BLIND_FORWARD_VELOCITY);
					}
					break;

					//    case STOP:
				case DROPPED_OBJECT:
					startingMove = true;
					fsmState = RoboFSM.MOVE_BACKWARD;
					//default:
					setVelocity(0.0, 0.0);
			}
		}
	}

	public void setVelocity(double rotVel, double transVel) {
		MotionMsg motionMsg = new MotionMsg();
		motionMsg.rotationalVelocity = rotVel;
		motionMsg.translationalVelocity = transVel;
		motionPub.publish(motionMsg);
	}


	/**
	 * <p>Moves from current (x, y, heading) towards target point (tX, tY) in
	 * a forward (1) or backward (-1) direction.<\p>
	 */
	private boolean moveTowardTarget(double x, double y, double heading,
			double tX, double tY, int direction) {
		System.out.println("  - current: x:" + x + " y:" + y);
		System.out.println("  - target: x:" + tX + " y:" + tY);

		// distance to target
		double tD = Math.sqrt((x-tX)*(x-tX) + (y-tY)*(y-tY));
		double tD1 = Math.hypot((x-tX), (x-tY));
		System.out.println("  Distance to target: " + tD + " td1: " + tD1);

		if (direction == DIR_BACKWARD) {
			heading = heading - Math.PI;
		}

		if (tD < TARGET_THRESHOLD) {
			return true;
		}
		else {
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
			System.out.println(" thetaError: " + thetaError);
			double rv = WHEEL_RV_GAIN * thetaError;
			System.out.println(" RV: " + rv);
			if (rv > WHEEL_MAX_RV) {
				rv = WHEEL_MAX_RV;
			}
			if (rv < -WHEEL_MAX_RV) {
				rv = -WHEEL_MAX_RV;
			}

			System.out.println(" clamped RV:" + rv);

			double tv = tD/TRANSPORT_DISTANCE * WHEEL_TV * direction;
			setVelocity(rv, tv);
		}
		return false;
	}


	private double target_hue_level = 0.03;
	private double hue_threshold= 0.008;
	private double saturation_level = 0.7;
	private double blob_size_threshold = 0.015;
	private double target_radius = 0.1;
	private double desired_fixation_distance = .5;
	private double translation_error_tolerance = .05;
	private double translation_velocity_gain = 0.5;
	private double translation_velocity_max = .25;
	private double rotation_error_tolerance = 0.2;
	private double rotation_velocity_gain = 0.1;
	private double rotation_velocity_max = 0.05;
	private boolean use_gaussian_blur = true;
	/**
	 * <p>Handle an image message. Perform blob tracking and
	 * servo robot towards target.</p>
	 *
	 * @param a received camera message
	 */
	public void handle(byte[] rawImage, int width, int height) {
		// on first camera message, create new BlobTracking instance
		if ( blobTrack == null ) {
			System.out.println("Blobtracking");
			blobTrack = new BlobTracking(width, height);

			blobTrack.targetHueLevel = target_hue_level;
			blobTrack.hueThreshold = hue_threshold;
			blobTrack.saturationLevel = saturation_level;
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
			System.out.println("done");
		}

		Image src = new Image(rawImage, width, height);
		Image dest = new Image(rawImage, width, height);
		blobTrack.apply(src, dest);

		org.ros.message.sensor_msgs.Image pubImage = new org.ros.message.sensor_msgs.Image();
		pubImage.width = width;
		pubImage.height = height;
		pubImage.encoding = "rgb8";
		pubImage.is_bigendian = 0;
		pubImage.step = width*3;
		pubImage.data = dest.toArray();
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

		switch (fsmState) {
			case VISUAL_SERVO_SEARCH: {
				System.out.println("VISUAL SERVO SEARCH");
				System.out.println("  range, bearing:" + blobTrack.targetRange + ", " +
						(blobTrack.targetBearing*180.0/Math.PI));

				if (Math.abs(blobTrack.targetRange-SEARCH_STANDOFF) < EPS_SEARCH_STANDOFF) {
					fsmState = RoboFSM.SET_ARM_FOR_GRASP;
					setVelocity(0.0, 0.0);
				} else {
					// move robot towards target
					System.out.println("  trans, rot:" + blobTrack.translationVelocityCommand + ", " +
							blobTrack.rotationVelocityCommand);
					setVelocity(blobTrack.rotationVelocityCommand, blobTrack.translationVelocityCommand);

				}
				break;
			}
			case VISUAL_SERVO_APPROACH: {
				System.out.println("VISUAL_SERVO APPROACH");
				// TODO
				//if (Math.abs(blobTrack.target
				// check distance to target and decrease standoff

				// if object is lost, go back to VSSEARCH

				blobTrack.desiredFixationDistance *= 0.75;
				// move robot towards target
				setVelocity(blobTrack.rotationVelocityCommand, blobTrack.translationVelocityCommand);

				break;
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
				System.out.println("    no msg received, sending to safe position, cmd=" + thetaSafe);
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
	 * GRIPPER CONTROLLER
	 */
	private class GripperController extends JointController{

		// GRIPPER STATES
		static final int GRIPPER_OPEN = 0;
		static final int GRIPPER_CLOSED = 1;
		static final int GRIPPER_OPENING = 2;
		static final int GRIPPER_CLOSING = 3;
    static final int GRIPPER_INITIALIZE = 4;

		// Measured points for calibration
		static final double servoPwmMin = 1300;//calibration : Gripper closed
		static final double thetaAtPwmMin = 0;
		static final double servoPwmMax = 1900;//calibration : Gripper open
		static final double thetaAtPwmMax = Math.PI/3.0;

		static final double maxThetaChange = 0.05;

		static final int currentAverageWindow = 3; // control-ticks
		static final double currentThresholdTime = 0.25; // seconds
		static final double currentThresholdValue = 0.50; // volts

		static final int gripperCurrentIdx = 1; //index
		static final double breakBeamThresholdTime = 0.5; // seconds
		static final double breakBeamThresholdValue = 0.5; // boolean

		double poseRelease = thetaMax; // radians
		double posePreshape = thetaMax; // radians
		double poseClosed = thetaMin;  // radians, such that break beam is active

		public GripperController() {
			super(servoPwmMin, servoPwmMax, thetaAtPwmMin, thetaAtPwmMax,
					maxThetaChange);
		}


		/* Return the desired gripper pose
		 * This implements a simple set of behaviors which are driven by the
		 * "state" of the world instead of internally held state. That is, the
		 * current pose is determined by the current sensor values and the current
		 * arm pose. We cheat a bit; because we don't actually have sensor feedback
		 * of the joint pose, so the physical pose of the arm/gripper can't
		 * maintain the state for us. Instead we use the commanded joint angle.
		 */

		@Override public void printState() {
			System.err.println("Gripper: Bump: "+bumpPressed);
			// + " Gripping: "+isGripping);
			super.printState();
		}

		public long step(ArmMsg msg) {
			if(LAB_PART == PART_2A) {return step_Part2a();}
			else if(LAB_PART == PART_3A) {return step_Part3a(msg);}
			else if(LAB_PART == PART_3B) {return step_Part3b(msg);}
			else {return step_Part3b(msg);}
		}

		public long step_Part3b(ArmMsg msg) {
			long returnVal = super.step();
			switch (fsmState) {

				case INITIALIZE_ARM: {
					returnVal = super.step(poseClosed);
					if (isAtDesired()) {
						System.out.println("   - Gripper initialized");
					}
					break;
				}

				case SET_ARM_FOR_GRASP: {
					System.out.println("SET ARM FOR GRASP");
					returnVal = super.step(posePreshape);

					if (isAtDesired()) {
						System.out.println("  - Gripper is ready for grasp");
					}
					break;
				}

				case BLIND_APPROACH: {
					System.out.println("BLIND_APPROACH - Gripper");
					if(bumpPressed) {
						System.out.println("  - GRIPPER BEAM BROKEN");
						fsmState = RoboFSM.GRASP_OBJECT;
					}
					break;
				}

				case VISUAL_SERVO_APPROACH: {
					System.out.println("VISUAL_SERVO_APPROACH - Gripper");
					if(bumpPressed) {
						System.out.println("  - GRIPPER BEAM BROKEN");
						fsmState = RoboFSM.GRASP_OBJECT;
					}
					break;
				}

				case WAIT_FOR_BREAKBEAM: {
					System.out.println("WAITING_FOR_BREAKBEAM");
					if(bumpPressed) {
						System.out.println("  - GRIPPER BEAM IS BROKEN");
						fsmState = RoboFSM.GRASP_OBJECT;
					}
					break;
				}

				case GRASP_OBJECT: {
					returnVal = super.step(poseClosed);
					if(isAtDesired()) {
						fsmState = RoboFSM.LIFT_OBJECT;
					}
					break;
				}

				case LIFT_OBJECT: {
					if(!bumpPressed) {
						fsmState = RoboFSM.DROPPED_OBJECT;
					}
					break;
				}

				case DROPPED_OBJECT: {
					System.out.println("DROPPED OBJECT");
					// if object dropped, reset_arm
					fsmState = RoboFSM.SET_ARM_FOR_GRASP;
					break;
				}

				case MOVE_FORWARD: {
					if(!bumpPressed) {
						fsmState = RoboFSM.DROPPED_OBJECT;
					}
					break;
				}

				case RELEASE_OBJECT: {
					System.out.println("RELEASING OBJECT");
					returnVal = super.step(poseRelease);

					if (isAtDesired()) {
						fsmState = RoboFSM.MOVE_BACKWARD;
					}
					break;
				}

			}

			return returnVal;
		}

		public long step_Part3a(ArmMsg msg) {
			long returnVal = super.step();
			switch (fsmState) {

				case INITIALIZE_ARM: {
					returnVal = super.step(poseClosed);
					if (isAtDesired()) {
						System.out.println("   - Gripper initialized");
					}
					break;
				}

				case SET_ARM_FOR_GRASP: {
					System.out.println("SET ARM FOR GRASP");
					returnVal = super.step(posePreshape);

					if (isAtDesired()) {
						System.out.println("  - Gripper is ready for grasp");
					}
					break;
				}

				case WAIT_FOR_BREAKBEAM: {
					System.out.println("WAITING_FOR_BREAKBEAM");
					if(bumpPressed) {
						System.out.println("  - GRIPPER BEAM IS BROKEN");
						fsmState = RoboFSM.GRASP_OBJECT;
					}
					break;
				}

				case GRASP_OBJECT: {
					returnVal = super.step(poseClosed);
					if(isAtDesired()) {
						fsmState = RoboFSM.LIFT_OBJECT;
					}
					break;
				}

				case LIFT_OBJECT: {
					if(!bumpPressed) {
						fsmState = RoboFSM.DROPPED_OBJECT;
					}
					break;
				}

				case DROPPED_OBJECT: {
					System.out.println("DROPPED OBJECT");
					// if object dropped, reset_arm
					fsmState = RoboFSM.SET_ARM_FOR_GRASP;
					break;
				}

				case MOVE_FORWARD: {
					if(!bumpPressed) {
						fsmState = RoboFSM.DROPPED_OBJECT;
					}
					break;
				}

				case RELEASE_OBJECT: {
					System.out.println("RELEASING OBJECT");
					returnVal = super.step(poseRelease);

					if (isAtDesired()) {
						fsmState = RoboFSM.MOVE_BACKWARD;
					}
					break;
				}

			}

			return returnVal;
		}

		/**
		 * <p>Step procedure for FSM in Part 2a<\p>
		 */
		public long step_Part2a() {
			long returnVal = 0;

		    if (state == GRIPPER_INITIALIZE) {
		        returnVal = super.step(poseClosed);
		    }
			// if state is gripper opening, continue opening until theta is met
			else if (state == GRIPPER_OPENING) {
				returnVal = super.step(poseRelease);

				if (isAtDesired()) {
					state = GRIPPER_OPEN;
				}
			}
			// if gripper state is closing, continue closing until theta is met
			else if (state == GRIPPER_CLOSING) {
				returnVal = super.step(poseClosed);

				if (isAtDesired()) {
					state = GRIPPER_CLOSED;
				}
			}
	        else if (state == GRIPPER_CLOSED) {
	        	returnVal = super.step(poseClosed);
	        }

			// if gripper state is open, begin closing
			else if (state == GRIPPER_OPEN) {
				state = GRIPPER_CLOSING;
			}

			// if gripper state is open, begin closing
			else if (state == GRIPPER_CLOSED) {
				state = GRIPPER_CLOSING;
			}

			return returnVal;
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
		static final double servoPwmMin = 750;
		static final double thetaAtPwmMin = 70*Math.PI/180;
		static final double servoPwmMax = 2300;
		static final double thetaAtPwmMax = -120*Math.PI/180;

		//static final double thetaPerPwmTick = (Math.PI) /
		//  (17265-pwmAtThetaZero); // calibration
		//static final double thetaPerPwmTick = (Math.PI) /
		//  (servoPwmMin - pwmAtThetaZero); // calibration

		static final double maxThetaChange = 0.03;

		//final double poseReleasing = thetaMax;    //radians
		final double poseRetracted = thetaMax;                           //radians
		//final double poseGrasp =
		final double poseExtended = thetaMin;

		final double poseGrasping = thetaAtPwmMin;                           //radians

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
			if(LAB_PART == PART_2A) {return step_Part2a();}
			else if(LAB_PART == PART_3A) {return step_Part3a(msg);}
			else if(LAB_PART == PART_3B) {return step_Part3b(msg);}
			else {return step_Part3b(msg);}
		}

		public long step_Part3b(ArmMsg msg) {
			long returnVal;

			switch (fsmState) {

				case INITIALIZE_ARM: {
					returnVal = super.step(poseRetracted);
					if (isAtDesired()) {
						System.out.println("  - Shoulder is initialized");
					}
					break;
				}

				case SET_ARM_FOR_GRASP: {
					returnVal = super.step(poseExtended);
					if(isAtDesired()) {
						System.out.println("  - Shoulder is at desired");
					}
					break;
				}

				case LIFT_OBJECT: {
					System.out.println("*** LIFT OBJECT ***");
					returnVal = super.step(poseRetracted);
					if (!bumpPressed){
						fsmState = RoboFSM.DROPPED_OBJECT;
					}
					if (isAtDesired()) {
						fsmState = RoboFSM.MOVE_FORWARD;
					}
					break;
				}

				case LOWER_OBJECT: {
					System.out.println("*** LOWER OBJECT ***");
					returnVal = super.step(poseExtended);
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

		public long step_Part3a(ArmMsg msg) {
			long returnVal;

			switch (fsmState) {

				case INITIALIZE_ARM: {
					returnVal = super.step(poseRetracted);
					if (isAtDesired()) {
						System.out.println("  - Shoulder is initialized");
					}
					break;
				}

				case SET_ARM_FOR_GRASP: {
					returnVal = super.step(poseGrasping);
					if(isAtDesired()) {
						System.out.println("  - Shoulder is at desired");
					}
					break;
				}

				case LIFT_OBJECT: {
					System.out.println("*** LIFT OBJECT ***");
					returnVal = super.step((thetaMax+thetaMin)/2.0);
					if (!bumpPressed){
						fsmState = RoboFSM.DROPPED_OBJECT;
					}
					if (isAtDesired()) {
						fsmState = RoboFSM.MOVE_FORWARD;
					}
					break;
				}

				case LOWER_OBJECT: {
					System.out.println("*** LOWER OBJECT ***");
					returnVal = super.step(poseGrasping);
					if (!bumpPressed){
						fsmState = RoboFSM.DROPPED_OBJECT;
					}
					if (isAtDesired()) {
						fsmState = RoboFSM.RELEASE_OBJECT;
					}
					break;
				}

				default : {
					returnVal = super.step();
				}
			}

			return returnVal;
		}

		/**
		 * <p>Step procedure for FSM in Part 2a<\p>
		 */
		public long step_Part2a(){
			long returnVal = 0;

            if (state == SHOULDER_INITIALIZE) {
        		returnVal = super.step((thetaMax+thetaMin)/2.0);

        	} 

        	else if (state == SHOULDER_FINALIZE) {
        		returnVal = super.step(thetaMax);

        	}
        	else if (state == SHOULDER_EXTENDING) {
				returnVal = super.step(poseExtended);

				if (isAtDesired()) {
					state = SHOULDER_EXTENDED;
				}
			} else if (state == SHOULDER_RETRACTING) {
				returnVal = super.step(poseRetracted);

				if (isAtDesired()) {
					state = SHOULDER_RETRACTED;
				}
			}

			else if (state == SHOULDER_EXTENDED) {
				state = SHOULDER_RETRACTING;
			}
			else if (state == SHOULDER_RETRACTED) {
				state = SHOULDER_EXTENDING;
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

		static final double servoPwmMin = 300;
		static final double thetaAtPwmMin = -110*Math.PI/180;
		static final double servoPwmMax = 2400;
		static final double thetaAtPwmMax = 110*Math.PI/180;

		static final double maxThetaChange = .05;    //Rad/sec

		double poseExtended = thetaMin;
		double poseRetracted = thetaMax;
		double poseGrasp = -0*Math.PI/180;
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
			if(LAB_PART == PART_2A) {return step_Part2a();}
			else if(LAB_PART == PART_3A) {return step_Part3a(msg);}
			else if(LAB_PART == PART_3B) {return step_Part3b(msg);}
			else {return step_Part3b(msg);}
		}


		public long step_Part3b(ArmMsg msg) {
			long returnVal;

			switch (fsmState) {

				case INITIALIZE_ARM: {
					returnVal = super.step(poseRetracted);
					if (isAtDesired()) {
						System.out.println("  - Wrist is initialized");
					}
					break;
				}

				case SET_ARM_FOR_GRASP: {
					returnVal = super.step(poseGrasp);

					if(isAtDesired()) {
						System.out.println("  - Wrist is ready for grasp");
					}

					break;
				}

				case LOWER_OBJECT: {
					returnVal = super.step(poseGrasp);
					if ( isAtDesired() ) {
						System.out.println("  - Wrist is lowered");
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

		public long step_Part3a(ArmMsg msg) {
			long returnVal;

			switch (fsmState) {

				case INITIALIZE_ARM: {
					returnVal = super.step(poseRetracted);
					if (isAtDesired()) {
						System.out.println("  - Wrist is initialized");
					}
					break;
				}

				case SET_ARM_FOR_GRASP: {
					returnVal = super.step(poseGrasp);

					if(isAtDesired()) {
						System.out.println("  - Wrist is ready for grasp");
					}

					break;
				}

				case LOWER_OBJECT: {
					returnVal = super.step(poseGrasp);
					if ( isAtDesired() ) {
						System.out.println("  - Wrist is lowered");
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


		/**
		 * <p>Step procedure for FSM in Part 2a<\p>
		 */
		public long step_Part2a() {
			long returnVal = 0;

		    if (state == WRIST_INITIALIZE) {
		        returnVal = super.step((thetaMax+thetaMin)/2.0);
		    } else if (state == WRIST_EXTENDING) {
				returnVal = super.step(poseExtended);
				if (isAtDesired()) {
					state = WRIST_EXTENDED;
				}
			} else if (state == WRIST_RETRACTING) {
				returnVal = super.step(poseRetracted);

				if (isAtDesired()) {
					state = WRIST_RETRACTED;
				}
			} else if (state == WRIST_EXTENDED) {
				state = WRIST_RETRACTING;
			}
			else if (state == WRIST_RETRACTED) {
				state = WRIST_EXTENDING;
			}

			return returnVal;
		}

	}

	@SuppressWarnings("unused")
	private void printControllerState() {
		System.err.println("---- CONTROLLER STATE ----");
		gripperControl.printState();
		shoulderControl.printState();
		wristControl.printState();
	}


	/**
	 * <p>Given joint angles, find (x,y,theta) position of
	 * end effector<\p>
	 */
	public void forwardKinematics(double theta1, double theta2) {
    double L = 0.239, l = 0.135;
		System.out.println("Compute Forward Kinematics for:");
		System.out.println("   l1:" + L + " theta1:" + theta1);
		System.out.println("   l2:" + l + " theta2:" + theta2);

		double theta = theta1 + theta2;
		double x = L*Math.cos(theta1) + l*Math.cos(theta);
		double z = L*Math.sin(theta1) + l*Math.sin(theta);

		System.out.println("----------------------------------------");
		System.out.println("   X:" + x + " Y:" + z + " Theta:" + theta);
	}


	/**
	 * <p>Given end effector position, find joint angles<\p>
	 */
	public void inverseKinematics(double x, double z) {
		System.out.println("Compute Inverse Kinematics for:");
		System.out.println("   x:" + x + " z:" + z);
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
    this.bumpPressed = value;
  }

}
