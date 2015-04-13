package gui_msgs;

public interface GUILineMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gui_msgs/GUILineMsg";
  static final java.lang.String _DEFINITION = "float64 lineA\nfloat64 lineB\nfloat64 lineC\nColorMsg color";
  double getLineA();
  void setLineA(double value);
  double getLineB();
  void setLineB(double value);
  double getLineC();
  void setLineC(double value);
  gui_msgs.ColorMsg getColor();
  void setColor(gui_msgs.ColorMsg value);
}
