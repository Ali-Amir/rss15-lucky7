package rss_msgs;

public interface MotionMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rss_msgs/MotionMsg";
  static final java.lang.String _DEFINITION = "float64 translationalVelocity\nfloat64 rotationalVelocity";
  double getTranslationalVelocity();
  void setTranslationalVelocity(double value);
  double getRotationalVelocity();
  void setRotationalVelocity(double value);
}
