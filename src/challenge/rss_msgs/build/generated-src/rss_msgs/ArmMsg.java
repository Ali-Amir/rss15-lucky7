package rss_msgs;

public interface ArmMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rss_msgs/ArmMsg";
  static final java.lang.String _DEFINITION = "int64[8] pwms";
  long[] getPwms();
  void setPwms(long[] value);
}
