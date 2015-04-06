package LocalNavigation;

public class SonarReading {
  
  public double range;
  public int sonar;
  public boolean isObstacle;

  SonarReading(int _sonar, double _range, boolean _isObstacle) {
    sonar = _sonar;
    range = _range;
    isObstacle = _isObstacle;
  }
}
