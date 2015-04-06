package LocalNavigation;

public class Robot {

  public double x;
  public double y;
  public double theta;
  public boolean isReady = false;


  public void reset(double _x, double _y, double _theta) {
    x = _x;
    y = _y;
    theta = _theta;
    isReady = true;
  }
}
