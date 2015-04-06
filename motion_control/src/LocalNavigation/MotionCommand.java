package LocalNavigation;

import java.awt.Color;

import java.util.ArrayList;
import java.util.List;

public abstract class MotionCommand {

  protected double x_start;
  protected double y_start;
  protected double theta_start;
  protected double ux;
  protected double uy;

  public void init(double x, double y, double theta) {
    x_start = x;
    y_start = y;
    theta_start = theta;
    ux = Math.cos(theta);
    uy = Math.sin(theta);
  }

  public abstract double getTranslationalVelocity(double x, double y,
                                                  double theta);

  public abstract double getRotationalVelocity(double x, double y,
                                               double theta);

  public abstract boolean isDone(double x, double y, double theta);

}
