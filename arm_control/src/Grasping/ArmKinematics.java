package Grasping;

import org.ros.message.rss_msgs.ArmMsg;

/**
 * <p>Part 2b: Arm Kinematics - Compute inverse kinematics for given
 * end-effector position and drive arm to resulting joint angles.  First
 * initialize arm, then move to desired position from computed joint
 * angles.<\p>
 **/

public class ArmKinematics extends GraspingFSM {

  /**
   * <p>X-coord of arm origin with respect to robot origin (meters)<\p>
   */
  static final double ARM_ORIGIN_X = 0.085;

  /**
   * <p>Z-coord of arm origin with respect to robot origin (meters)<\p>
   */
  static final double ARM_ORIGIN_Z = 0.26;

  /**
   * <p>Length of link 1 of the arm (shoulder link) (meters)<\p>
   */
  static final double ARM_LINK_1 = 0.245;

  /**
   * <p>Length of link 2 of the arm (wrist link) (meters)<\p>
   */
  static final double ARM_LINK_2 = 0.175;

  /**
   * <p>Stores desired shoulder theta, used in MOVE_TO_DESIRED state<\p>
   */
  double shoulderTheta;

  /**
   * <p>Stores desired wrist theta, used in MOVE_TO_DESIRED state<\p>
   */
  double wristTheta;

  /**
   * <p>Constructor for ArmKinematics object<\p>
   */
  public ArmKinematics () {
    super();

    // (A) Use for testing forward kinematics. Drive arm to given thetas
    // and output (x,z,theta) of end-effector.
    /*
    // desired joint angles (ex. 30 degrees for each link)
    shoulderTheta = 30*Math.PI/180;
    wristTheta = 30*Math.PI/180;
    System.out.println("Moving arm to: " + shoulderTheta + ", " + wristTheta);

    // computer forward kinematics
    forwardKinematics(ARM_LINK_1, ARM_LINK_2, shoulderTheta, wristTheta);
    */

    // (B) Use for testing inverse kinematics.
    // pick end-effector position (example)
    double x = 0.38;
    double z = 0.53;

    // compute inverse kinematics and drive arm
    inverseKinematics(x, z, ARM_LINK_1, ARM_LINK_2);
  }

  /**
   * <p>Gripper handler for arm message, handles gripper motion<\p>
   */
  @Override
  public long gripperHandler(ArmMsg msg) {
    // keep gripper closed
    return gripperControl.step(gripperControl.poseClosed, msg);
  }

  /**
   * <p>Wrist handler for arm message, handles wrist motion<\p>
   */
  @Override
  public long wristHandler(ArmMsg msg) {
    return wristControl.step(wristTheta, msg);
  }

  /**
   * <p>Shoulder handler for arm message, handles shoulder motion<\p>
   */
  @Override
  public long shoulderHandler(ArmMsg msg) {
    return shoulderControl.step(shoulderTheta, msg);
  }

  /**
   * <p>Given joint angles, find (x,y,theta) position of
   * end effector<\p>
   */
  public void forwardKinematics(double l1, double l2, double theta1,
                                double theta2) {
    System.out.println("Compute Forward Kinematics for:");
    System.out.println("   l1:" + l1 + " theta1:" + theta1);
    System.out.println("   l2:" + l2 + " theta2:" + theta2);

    double theta = theta1 + theta2;
    double x = l1*Math.cos(theta1) + l2*Math.cos(theta);
    double z = l1*Math.sin(theta1) + l2*Math.sin(theta);

    System.out.println("----------------------------------------");
    System.out.println("Position relative to arm origin:");
    System.out.println("   X:" + x + " Z:" + z + " Theta:" + theta);

    // adjust according to robot origin
    double x_robot = x + ARM_ORIGIN_X;
    double z_robot = z + ARM_ORIGIN_Z;
    System.out.println("Position relative to robot origin:");
    System.out.println("   X:" + x_robot + " Z:" + z_robot + " Theta:"
                       + theta);
  }

  /**
   * <p>Given end effector position, find joint angles<\p>
   */
  public void inverseKinematics(double x, double z, double l1, double l2) {
    System.out.println("Compute Inverse Kinematics for:");
    System.out.println("   x:" + x + " z:" + z + " l1:" + l1 + " l2:" + l2);

    // first adjust (x,z) to arm origin for calculations
    x = x - ARM_ORIGIN_X;
    z = z - ARM_ORIGIN_Z;
    System.out.println("Adjusted Coords for Arm Origin:");
    System.out.println("   x:" + x + " z:" + z + " l1:" + l1 + " l2:" + l2);

    double p = -2*x*l2;
    double q = -2*z*l2;
    double r = x*x + z*z - l1*l1 + l2*l2;

    System.out.println("   P:" + p + " Q:" + q + " R:" + r);

    double a = r-p;
    double b = 2*q;
    double c = r+p;

    System.out.println("   A:" + a + " B:" + b + " C:" + c);

    // Two arm configuration solutions, a and b
    double t_a = (-b + Math.sqrt(b*b - 4*a*c)) / (2*a);
    double t_b = (-b - Math.sqrt(b*b - 4*a*c)) / (2*a);

    System.out.println("   t_a:" + t_a + " t_b:" + t_b);

    double theta_a, theta_a_1, theta_a_2;
    double theta_b, theta_b_1, theta_b_2;

    // t=tan(u)/2, u=atan(2t)
    theta_a = Math.atan(2*t_a);
    theta_b = Math.atan(2*t_b);

    // shoulder angle is _1, wrist angle is _2
    theta_a_1 = Math.atan2((z-l2*Math.sin(theta_a)),
                           (x - l2*Math.cos(theta_a)));
    theta_a_2 = theta_a - theta_a_1;

    theta_b_1 = Math.atan2((z-l2*Math.sin(theta_b)),
                           (x - l2*Math.cos(theta_b)));
    theta_b_2 = theta_b - theta_b_1;

    System.out.println("----------------------------------------");
    System.out.println("   ThetaA:" + theta_a + " a_1:" + theta_a_1 + " a_2:" +
                       theta_a_2);
    System.out.println("   ThetaB:" + theta_b + " b_1:" + theta_b_1 + " b_2:" +
                       theta_b_2);

    // Choose one solution, we can choose that our wrist angles are (+)
    if(theta_a_2 > 0) {
      shoulderTheta = theta_a_1;
      wristTheta = theta_a_2;
    }
    else {
      shoulderTheta = theta_a_1;
      wristTheta = theta_a_2;
    }

    System.out.println("Moving arm to: shoulder:" + shoulderTheta +
                       " wrist:" + wristTheta);
  }
}
