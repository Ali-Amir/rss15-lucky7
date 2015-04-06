package LocalNavigation;

import java.awt.Color;

import java.util.ArrayList;
import java.util.List;

public class PersistentCommand extends MotionCommand {

  protected double transVel;
  protected double rotVel;

  public PersistentCommand(double tv, double rv) {
    transVel = tv;
    rotVel = rv;
  }

  public double getTranslationalVelocity(double x, double y,
                                         double theta) {
    return transVel;
  }

  public double getRotationalVelocity(double x, double y,
                                      double theta) {
    return rotVel;
  }

  public boolean isDone(double x, double y, double theta) {
    // TODO
    return false;
  }
}
