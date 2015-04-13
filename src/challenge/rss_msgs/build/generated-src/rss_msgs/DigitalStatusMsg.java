package rss_msgs;

public interface DigitalStatusMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rss_msgs/DigitalStatusMsg";
  static final java.lang.String _DEFINITION = "bool[8] slow\nbool[8] fast";
  boolean[] getSlow();
  void setSlow(boolean[] value);
  boolean[] getFast();
  void setFast(boolean[] value);
}
