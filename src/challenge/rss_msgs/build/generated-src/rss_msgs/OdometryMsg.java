package rss_msgs;

public interface OdometryMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rss_msgs/OdometryMsg";
  static final java.lang.String _DEFINITION = "float64 x\nfloat64 y\nfloat64 theta";
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
  double getTheta();
  void setTheta(double value);
}
