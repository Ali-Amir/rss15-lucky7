package gui_msgs;

public interface GUIPointMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gui_msgs/GUIPointMsg";
  static final java.lang.String _DEFINITION = "float64 x\nfloat64 y\nint64 shape\nColorMsg color";
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
  long getShape();
  void setShape(long value);
  gui_msgs.ColorMsg getColor();
  void setColor(gui_msgs.ColorMsg value);
}
