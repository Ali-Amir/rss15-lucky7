package Grasping;

import org.ros.message.rss_msgs.ArmMsg;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.namespace.GraphName;

/**
 * Initializes arm, when ready leaves in state ARM INITIALIZED
 **/

public abstract class GraspingFSM implements NodeMain {

	/**
	 * <p>Shoulder joint array index.  Note, current sense is on Orc port 0,
	 * which we want for shoulder.<\p>
	 */
	static final int SHOULDER_INDEX = 0;

	/**
	 * <p>Gripper joint array index.  Note, current sense is on Orc port 1,
	 * which we want for gripper.<\p>
	 */
	static final int GRIPPER_INDEX = 1;

	/**
	 * <p>Wrist joint array index<\p>
	 */
	static final int WRIST_INDEX = 2;

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
	JointController[] armControl = {shoulderControl, gripperControl,
			wristControl};

	/**
	 * <p>Commanded servo positions to desired set point<\p>
	 */
	public long[] armTheta = new long[]{0,0,0};

	//public static final int NULL_STATE = -1;

	/**
	 * <p>FSM state: arm initial position.  The initial position is expected in
	 * fully retracted state, shoulder back, arm back, gripper closed.<\p>
	 */
	static final int INITIALIZE_ARM = -2;
	static final int ARM_INITIALIZED = -1;

	/**
	 * <p>Stores the current FSM state<\p>
	 */
	int fsmState = INITIALIZE_ARM;

	private Publisher<ArmMsg> armPub;

	/**
	 * <p>Constructor for Grasping object.  Be sure to set an fsmState before
	 * calling constructor.<\p>
	 */
	public GraspingFSM () {
	}

        private Subscriber<org.ros.message.rss_msgs.ArmMsg> armSub;

	/**
	 * <p> Entry point for ros code. Subscribes node to arm messages.</p>
	 */
	@Override
	public void onStart(Node node){
	    armSub = node.newSubscriber("rss/ArmStatus", "rss_msgs/ArmMsg");
	    armSub.addMessageListener(new ArmListener(this));
	    armPub = node.newPublisher("command/Arm", "rss_msgs/ArmMsg");
	}

	/**
	 * <p> Called when ros shuts this node down.</p>
	 */
	@Override
	public void onShutdown(Node node){
	}

    @Override public void onShutdownComplete(Node node) {
    }

    @Override public GraphName getDefaultNodeName() {
	return new GraphName("rss/graspingfsm");
    }

	/**
	 * <p>Handle an ArmMessage<\p>
	 */
	public void handle(ArmMsg msg) {

		// handle initializing arm
		if (fsmState == INITIALIZE_ARM) {

			armTheta[GRIPPER_INDEX] =
					gripperControl.step(gripperControl.poseClosed, msg);
			armTheta[WRIST_INDEX] =
					wristControl.step(wristControl.poseRetracted, msg);
			armTheta[SHOULDER_INDEX] =
					shoulderControl.step(shoulderControl.poseRetracted, msg);

			if ( gripperControl.isAtDesired()
					&& wristControl.isAtDesired()
					&& shoulderControl.isAtDesired() ) {
				System.out.println("Arm is initialized");
				fsmState = ARM_INITIALIZED;
			}
		}
		else {
			armTheta[GRIPPER_INDEX] = gripperHandler(msg);
			armTheta[WRIST_INDEX] = wristHandler(msg);
			armTheta[SHOULDER_INDEX] = shoulderHandler(msg);
		}

		// Send arm pose to the Orc
		ArmMsg newMsg = new ArmMsg();
		newMsg.pwms = armTheta;
		armPub.publish(newMsg);
	}

	public abstract long gripperHandler(ArmMsg msg);

	public abstract long wristHandler(ArmMsg msg);

	public abstract long shoulderHandler(ArmMsg msg);

}
