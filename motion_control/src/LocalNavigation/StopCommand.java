package LocalNavigation;

import java.awt.Color;

import java.util.ArrayList;
import java.util.List;

public class StopCommand extends MotionCommand {

  protected final static int MAX_STOP_COUNTER = 3;

  protected double x_prev;
  protected double y_prev;
  protected double theta_prev;
  protected int stopCounter;

  public StopCommand() {
    stopCounter = 0;
  }

  @Override
  public void init(double x, double y, double theta) {
    super.init(x, y, theta);
    x_prev = x;
    y_prev = y;
    theta_prev = theta;
  }

  public double getTranslationalVelocity(double x, double y,
                                         double theta) {
    return 0.0;
  }

  public double getRotationalVelocity(double x, double y,
                                      double theta) {
    return 0.0;
  }

  public boolean isDone(double x, double y, double theta) {
    double xx = x_prev;
    double yy = y_prev;
    double theta_t = theta_prev;

    x_prev = x;
    y_prev = y;
    theta_prev = theta;

    double dt = Math.abs((theta-theta_t)%(2.0*Math.PI));
    double THRESH = Math.PI/180.0*0.5;
    if (Math.sqrt((x - xx)*(x - xx) +
                  (y - yy)*(y - yy)) < 0.05 &&
        (dt < THRESH || dt > 2.0*Math.PI-THRESH)) {
      ++stopCounter;
      if (stopCounter > MAX_STOP_COUNTER) {
        System.out.println("Stop successfull");
        return true;
      }
    }
    return false;
  }
}
