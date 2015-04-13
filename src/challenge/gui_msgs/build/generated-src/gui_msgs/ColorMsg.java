package gui_msgs;

public interface ColorMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gui_msgs/ColorMsg";
  static final java.lang.String _DEFINITION = "int64 r\nint64 g\nint64 b";
  long getR();
  void setR(long value);
  long getG();
  void setG(long value);
  long getB();
  void setB(long value);
}
