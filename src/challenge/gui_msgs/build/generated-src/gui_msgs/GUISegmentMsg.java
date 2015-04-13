package gui_msgs;

public interface GUISegmentMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gui_msgs/GUISegmentMsg";
  static final java.lang.String _DEFINITION = "float64 startX\nfloat64 endX\nfloat64 startY\nfloat64 endY\nColorMsg color\n";
  double getStartX();
  void setStartX(double value);
  double getEndX();
  void setEndX(double value);
  double getStartY();
  void setStartY(double value);
  double getEndY();
  void setEndY(double value);
  gui_msgs.ColorMsg getColor();
  void setColor(gui_msgs.ColorMsg value);
}
