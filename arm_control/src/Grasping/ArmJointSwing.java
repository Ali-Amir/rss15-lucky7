package Grasping;

import org.ros.message.rss_msgs.ArmMsg;

/**
 * <p>Lab 8: Part 2A - Move all arm servos through complete range of motion
 * simultaneously.<\p>
 **/

public class ArmJointSwing extends GraspingFSM {

  /**
   * <p>Gripper FSM State: gripper fully open.<\p>
   */
  static final int GRIPPER_OPEN = 0;

  /**
   * <p>Gripper FSM State: gripper closed.<\p>
   */
  static final int GRIPPER_CLOSED = 1;

  /**
   * <p>Gripper FSM State: gripper in process of opening.<\p>
   */
  static final int GRIPPER_OPENING = 2;

  /**
   * <p>Gripper FSM State: gripper in process of closing.<\p>
   */
  static final int GRIPPER_CLOSING = 3;

  /**
   * <p>Wrist FSM State: wrist is in fully back, retracted position.<\p>
   */
  static final int WRIST_RETRACTED = 4;

  /**
   * <p>Wrist FSM State: wrist is forward, in extended position.<\p>
   */
  static final int WRIST_EXTENDED = 5;

  /**
   * <p>Wrist FSM State: wrist in process of retracting.<\p>
   */
  static final int WRIST_RETRACTING = 6;

  /**
   * <p>Wrist FSM State: wrist in process of extending.<\p>
   */
  static final int WRIST_EXTENDING = 7;

  /**
   * <p>Shoulder FSM State: shoulder in upright, retracted position.<\p>
   */
  static final int SHOULDER_RETRACTED = 8;

  /**
   * <p>Shoulder FSM State: shoulder extended forward.<\p>
   */
  static final int SHOULDER_EXTENDED = 9;

  /**
   * <p>Shoulder FSM State: shoulder in process of retracting.<\p>
   */
  static final int SHOULDER_RETRACTING = 10;

  /**
   * <p>Shoulder FSM State: shoulder in process of extending.<\p>
   */
  static final int SHOULDER_EXTENDING = 11;

  /**
   * <p>Stores FSM State of gripper<\p>
   */
  public int gripper_state;

  /**
   * <p>Stores FSM State of wrist<\p>
   */
  public int wrist_state;

  /**
   * <p>Stores FSM State of shoulder<\p>
   */
  public int shoulder_state;

  /**
   * <p>Constructor for ArmJointSwing object<\p>
   */
  public ArmJointSwing() {
    // initialize GraspingFSM and arm state
    super();

    // Set initial state, arm in fully retracted, closed position
    gripper_state = GRIPPER_CLOSED;
    wrist_state = WRIST_RETRACTED;
    shoulder_state = SHOULDER_RETRACTED;
  }

  /**
   * <p>Gripper handler for ArmMessage<\p>
   */
  @Override
  public long gripperHandler(ArmMsg msg) {
    long returnVal = 0;

    // if state is gripper opening, continue opening until theta is met
    if (gripper_state == GRIPPER_OPENING) {
      returnVal = gripperControl.step(gripperControl.poseRelease, msg);

      if (gripperControl.isAtDesired()) {
        gripper_state = GRIPPER_OPEN;
      }
    }
    // if gripper state is closing, continue closing until theta is met
    else if (gripper_state == GRIPPER_CLOSING) {
      returnVal = gripperControl.step(gripperControl.poseClosed, msg);

      if (gripperControl.isAtDesired()) {
        gripper_state = GRIPPER_CLOSED;
      }
    }

    // if gripper state is open, begin closing
    if (gripper_state == GRIPPER_OPEN) {
      gripper_state = GRIPPER_CLOSING;
    }
    else if (gripper_state == GRIPPER_CLOSED) {
      gripper_state = GRIPPER_OPENING;
    }

    return returnVal;
  }


  /**
   * <p>Wrist handler for ArmMessage<\p>
   */
  @Override
  public long wristHandler(ArmMsg msg) {
    long returnVal = 0;

    if (wrist_state == WRIST_EXTENDING) {
      returnVal = wristControl.step(wristControl.poseExtended, msg);

      if (wristControl.isAtDesired()) {
        wrist_state = WRIST_EXTENDED;
      }
    }
    else if (wrist_state == WRIST_RETRACTING) {
      returnVal = wristControl.step(wristControl.poseRetracted, msg);

      if (wristControl.isAtDesired()) {
        wrist_state = WRIST_RETRACTED;
      }
    }

    if (wrist_state == WRIST_EXTENDED) {
      wrist_state = WRIST_RETRACTING;
    }
    else if (wrist_state == WRIST_RETRACTED) {
      wrist_state = WRIST_EXTENDING;
    }

    return returnVal;

  }


  /**
   * <p>Shoulder Handler for ArmMessage<\p>
   */
  @Override
  public long shoulderHandler(ArmMsg msg) {
    long returnVal = 0;

    if (shoulder_state == SHOULDER_EXTENDING) {
      returnVal = shoulderControl.step(shoulderControl.poseExtended, msg);

      if (shoulderControl.isAtDesired()) {
        shoulder_state = SHOULDER_EXTENDED;
      }
    }
    else if (shoulder_state == SHOULDER_RETRACTING) {
      returnVal = shoulderControl.step(shoulderControl.poseRetracted, msg);

      if (shoulderControl.isAtDesired()) {
        shoulder_state = SHOULDER_RETRACTED;
      }
    }

    if (shoulder_state == SHOULDER_EXTENDED) {
      shoulder_state = SHOULDER_RETRACTING;
    }
    else if (shoulder_state == SHOULDER_RETRACTED) {
      shoulder_state = SHOULDER_EXTENDING;
    }

    return returnVal;
  }
}
