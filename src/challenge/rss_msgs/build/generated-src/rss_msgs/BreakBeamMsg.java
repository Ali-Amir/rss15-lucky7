package rss_msgs;

public interface BreakBeamMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rss_msgs/BreakBeamMsg";
  static final java.lang.String _DEFINITION = "bool beamBroken";
  boolean getBeamBroken();
  void setBeamBroken(boolean value);
}
