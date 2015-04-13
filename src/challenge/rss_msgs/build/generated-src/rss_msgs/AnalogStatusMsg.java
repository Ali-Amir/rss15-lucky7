package rss_msgs;

public interface AnalogStatusMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rss_msgs/AnalogStatusMsg";
  static final java.lang.String _DEFINITION = "float64[8] values";
  double[] getValues();
  void setValues(double[] value);
}
