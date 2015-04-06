package LocalNavigation;

import org.ros.message.rss_msgs.BumpMsg;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.message.rss_msgs.SonarMsg;
import org.ros.node.topic.Publisher;
import org.ros.node.Node;

enum State {
  STOP_ON_BUMP, ALIGN_ON_BUMP, ALIGNING, ALIGNED, ALIGNED_AND_ROTATING,
  ALIGNED_AND_ROTATED, BACKING_UP, FINDING_WALL, TRACKING_WALL, WALL_ENDED,
  GO_AROUND_WALL, SENSING_WALL, WALL_SENSED, DONE
}

enum SonarState {
  NOT_OBSERVING, OBSERVING, TRACKING
}

public class StateHandler {

  State state = State.ALIGN_ON_BUMP;
  MotionCommander motionCommander;
  ObstacleModel obstacleModel;
  SonarModel[] sonar;
  WallModel wallModel;

  double desired_dist;
  double desired_theta;

  Robot robot;
  double paramA;
  double paramB;
  double paramC;

  double turned = 0;
  double prevTheta;
  double newTheta;
  boolean initial = true;

  SonarState sonarState = SonarState.NOT_OBSERVING;

  double alpha = .7;

  int[] clearedWallSonarsCount = new int[2];
  boolean[] seenWallSonars = new boolean[2];
  double perc = .20;

  double trans_err;
  double rotate_err;


  StateHandler(State initState, Node initNode, Robot robotPtr) {
    state = initState;
    robot = robotPtr;
    motionCommander = new MotionCommander(initNode, robotPtr);
    obstacleModel = new ObstacleModel();
    wallModel = new WallModel(initNode);
    sonar = new SonarModel[]{new SonarModel(robotPtr, wallModel, "back"),
                             new SonarModel(robotPtr, wallModel, "front")};
  }

  State getState() {
    return state;
  }

  void setState(State newState) {
    if (newState != state) {
      System.out.println("Changing state from " + state.toString() +
                         " to " + newState.toString());
      state = newState;
    }
  }

  protected boolean shouldAddObservation(int sonarId) {
    if (this.getState() == State.BACKING_UP ||
        this.getState() == State.TRACKING_WALL) {
      return !sonar[sonarId].hasClearedWall() &&
             sonar[sonarId].isReadingWall;
    }
    return false;
  }

  // TODO: EXPERIMENTATION MODULE
  protected synchronized void dummyCommand() {
    motionCommander.command(new PersistentCommand(0.2));
  }

  protected synchronized void step() {
    switch (this.getState()) {
      case ALIGN_ON_BUMP: {
        if (obstacleModel.isObstacleComplete()) {
          motionCommander.command(new StopCommand());
          this.setState(State.DONE);
          step();
        }
        break;
      }
        
      case GO_AROUND_WALL: {
        double d = wallModel.getDistanceToWall();
        double c = d + MotionCommander.BASE_WIDTH_IN_M/2.0;
        double l = 0.3302;
        double r = (c*c+l*l)/(2.0*l);
        double v = 0.03;
        double w = v / r;
        MotionCommand[] comms = {new RotateCommand(Math.PI/2.0),
                                 new PersistentCommand(v, w)};
        motionCommander.command(comms);
        this.setState(State.ALIGN_ON_BUMP);
        break;
      }

      case ALIGNED: {
        motionCommander.command(new StopCommand());
        if (!initial) {
          newTheta = robot.theta - Math.PI/2;
          if (newTheta<0) {
            newTheta+=Math.PI*2;
          }
          turned+= newTheta-prevTheta;
          
          if (turned>Math.PI*2-.3) {
            this.setState(State.DONE);
            motionCommander.command(new StopCommand());
          } else {
            this.setState(State.ALIGNED_AND_ROTATING);
            MotionCommand[] comms = {new TranslateCommand(-0.4),
                                     new StopCommand(),
                                     new RotateCommand(-Math.PI/2.0),
                                     new StopCommand()};
            motionCommander.command(comms);
          }
        } else {
          initial = false;
          MotionCommand[] comms = {new TranslateCommand(-0.3),
                                   new StopCommand(),
                                   new RotateCommand(-Math.PI/2.0),
                                   new StopCommand()};
          motionCommander.command(comms);
          this.setState(State.ALIGNED_AND_ROTATING);
          step();
        }
        break;
      }

      case ALIGNED_AND_ROTATING: {
        if (motionCommander.isDone()) {
          this.setState(State.ALIGNED_AND_ROTATED);
          step();
        }
        break;
      }

      case ALIGNED_AND_ROTATED: {
        sonarState = SonarState.OBSERVING;
        desired_theta = robot.theta;
        wallModel.resetModel();
        this.setState(State.SENSING_WALL);
        step();
        break;
      }

      case SENSING_WALL: {
        if (wallModel.isWallStable()) {
          this.setState(State.WALL_SENSED);
          step();
        }
        break;
      }

      case WALL_SENSED: {
        sonar[0].reset();
        sonar[1].reset();
        sonarState = SonarState.TRACKING;
        this.setState(State.BACKING_UP);
        step();
        break;
      }

      case WALL_ENDED: {
        obstacleModel.addWallSegment(wallModel.getWallSegment());
        sonarState = SonarState.NOT_OBSERVING;
        prevTheta = robot.theta;
        this.setState(State.GO_AROUND_WALL);
        step();
        break;
      }

      case BACKING_UP: {
        if (!sonar[0].hasClearedWall() || !sonar[1].hasClearedWall()) {
          MotionCommand[] comm = getWallTrackingComms(false);
          motionCommander.command(comm);
        } else {
          MotionCommand[] comm = {new StopCommand()};
          motionCommander.command(comm);
          this.setState(State.FINDING_WALL);
          step();
        }
        break;
      }

      case FINDING_WALL: {
        if (sonar[0].isReadingWall || sonar[1].isReadingWall) {
          desired_theta = robot.theta;
          sonar[0].reset();
          sonar[1].reset();
          this.setState(State.TRACKING_WALL);
          step();
        } else {
          motionCommander.command(new PersistentCommand(0.04, 0.0));
        }
        break;
      }

      case TRACKING_WALL: {
        if (sonar[0].hasClearedWall() && sonar[1].hasClearedWall()) {
          wallModel.drawWall();
          sonarState = SonarState.OBSERVING;
          motionCommander.command(new StopCommand());
          this.setState(State.WALL_ENDED);
          step();
        } else {
          MotionCommand[] comm = getWallTrackingComms(true);
          motionCommander.command(comm);
        }
        break;
      }
    }
  }

  void handleOdometryUpdate() {
    motionCommander.step();
    step();
  }

  void handleSonarUpdate(SonarReading reading) {
    sonar[reading.sonar].updateSonar(reading);

    double p[] = LocalNavigation.getCoordsOffset(
        robot,
        LocalNavigation.SONAR_REL_X[reading.sonar],
        LocalNavigation.SONAR_REL_Y[reading.sonar]+reading.range);
    if (this.getState() == State.SENSING_WALL ||
        shouldAddObservation(reading.sonar)) {
      wallModel.addObservation(p[0], p[1]);
      wallModel.updateDistance(robot.x, robot.y);
    }

    step();
  }

  void handleBumpMsg(BumpMsg msg) {
    if (this.getState() == State.ALIGN_ON_BUMP) {
      if (msg.left || msg.right) {
        this.setState(State.ALIGNING);
      }
    }

    if (this.getState() == State.ALIGNING) {
      if (msg.left && msg.right) {
        this.setState(State.ALIGNED);
      } else if (msg.left) {
        // Tell it to keep rotating left. First parameter is translational
        // velocity. Second parameter is rotational velocity.
//        MotionCommand[] comm = {new PersistentCommand(0.0, Math.PI/100.0)};
        motionCommander.command(new PersistentCommand(0.0, Math.PI/25.0));
      } else if (msg.right) {
        // Tell it to keep rotating right. First parameter is translational
        // velocity. Second parameter is rotational velocity.
//        MotionCommand[] comm = {new PersistentCommand(0.0, -Math.PI/100.0)};
        motionCommander.command(new PersistentCommand(0.0, -Math.PI/25.0));
      } else {
        // Tell it to keep moving forward. First parameter is translational
        // velocity. Second parameter is rotational velocity.
//        MotionCommand[] comm = {new PersistentCommand(0.05, 0.0)};
        motionCommander.command(new PersistentCommand(0.03, 0.0));
      }
    }

    if (state == State.STOP_ON_BUMP && (msg.left || msg.right)){
      // Command it to stop if we hit on something.
//      MotionCommand[] comm = {new StopCommand()};
      motionCommander.command(new StopCommand());
    }
    
    step();
  }

  public boolean isWall(double range){
    if (sonarState == SonarState.TRACKING){
      if (withinPercent(range, desired_dist, perc)){
        return true;
      } else {
        return false;
      }
    } else if (sonarState == SonarState.OBSERVING){
      return true;
    } else {
      return false;
    }
  }

/*
  void updateWallSonars(SonarReading reading){
    double p[] = LocalNavigation.getCoordsOffset(
        robot,
        LocalNavigation.SONAR_REL_X[reading.sonar],
        LocalNavigation.SONAR_REL_Y[reading.sonar]+reading.range);
    if (wallModel.isPartOfWall(reading.range, p[0], p[1])) {
      seenWallSonars[reading.sonar] = true;
    }

    if (clearedWallSonarsCount[reading.sonar] < WALL_CLEARED_THRESH) {
      if (seenWallSonars[reading.sonar]) {
        if (!wallModel.isPartOfWall(reading.range, p[0], p[1])) {
          System.out.println("///TEST//");
          System.out.println("Sonar: "+reading.sonar);
          System.out.println("Range: "+ reading.range);
          System.out.println("prevRange: "+ desired_dist);
          ++clearedWallSonarsCount[reading.sonar];
        } else {
          clearedWallSonarsCount[reading.sonar] = 0;
        }
      }
    }
  }
*/

  private boolean withinPercent(double value, double standard, double percent) {
    standard = Math.abs(standard);
    value = Math.abs(value);
    return (value > standard*(1-percent) && value < standard*(1+percent));
  }

  private double calcPerpDistance(){
    return Math.abs(paramA*robot.x + paramB*robot.y + paramC);
  }

  private void calcTransError(){
    trans_err = calcPerpDistance() - desired_dist;
  }

  private void calcRotateError(){
    rotate_err = robot.theta - desired_theta;
  }

  private MotionCommand[] getWallTrackingComms(boolean forward){
    calcTransError();
    calcRotateError();
    double rotate_offset = 0;
    if (trans_err>.05){
      rotate_offset+=Math.PI/100;
    } else if (trans_err<-.05){
      rotate_offset-=Math.PI/100;
    } 

    if (forward){
      
      MotionCommand[] comm = {new PersistentCommand(0.05, 0.0)};//-rotate_err+rotate_offset)};
      return comm;
    } else {
      // System.out.println("//////DIAGNOSTICS///////");
      // System.out.println("Desired Theta: " + desired_theta);
      // System.out.println("Current Theta: " + robot.theta);
      // System.out.println("Error : " + rotate_err);
      // System.out.println("Offset : " + rotate_offset);

      // System.out.println("Desired Dist : " + desired_dist);
      // System.out.println("Trans_Err : " + trans_err);
      MotionCommand[] comm = {new PersistentCommand(-0.05, 0.0)};//-rotate_err-rotate_offset)};
      return comm;
    }
    
  }

}


