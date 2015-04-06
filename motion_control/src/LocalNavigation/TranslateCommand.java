package LocalNavigation;

import java.awt.Color;

import java.util.ArrayList;
import java.util.List;

public class TranslateCommand extends MotionCommand {

  protected double delta_d;
  protected double ux;
  protected double uy;

  public TranslateCommand(double d_d) {
    delta_d = d_d;
  }

  public double getTranslationalVelocity(double x, double y,
                                         double theta) {
    if (isDone(x, y, theta)) {
      return 0.0;
    }

    double delta_cur = (x - x_start)*ux + (y - y_start)*uy;
    double v = (delta_d - delta_cur)*2.0;
    v = Math.min(v, 0.03);
    v = Math.max(v, -0.03);
    return v;
  }

  public double getRotationalVelocity(double x, double y,
                                      double theta) {
    return 0.0;
  }

  public boolean isDone(double x, double y, double theta) {
    return Math.abs(Math.sqrt((x - x_start)*(x - x_start) +
                    (y - y_start)*(y - y_start)) - Math.abs(delta_d)) < 0.05;
  }
}
