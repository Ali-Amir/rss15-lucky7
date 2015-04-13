package rss_msgs;

public interface EncoderMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rss_msgs/EncoderMsg";
  static final java.lang.String _DEFINITION = "int64 left\nint64 right";
  long getLeft();
  void setLeft(long value);
  long getRight();
  void setRight(long value);
}
