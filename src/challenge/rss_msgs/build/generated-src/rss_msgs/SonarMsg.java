package rss_msgs;

public interface SonarMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rss_msgs/SonarMsg";
  static final java.lang.String _DEFINITION = "float64 range\nbool isFront";
  double getRange();
  void setRange(double value);
  boolean getIsFront();
  void setIsFront(boolean value);
}
