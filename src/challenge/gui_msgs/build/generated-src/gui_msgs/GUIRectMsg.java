package gui_msgs;

public interface GUIRectMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gui_msgs/GUIRectMsg";
  static final java.lang.String _DEFINITION = "ColorMsg c\nfloat32 x\nfloat32 y\nfloat32 width\nfloat32 height\nint32 filled\n";
  gui_msgs.ColorMsg getC();
  void setC(gui_msgs.ColorMsg value);
  float getX();
  void setX(float value);
  float getY();
  void setY(float value);
  float getWidth();
  void setWidth(float value);
  float getHeight();
  void setHeight(float value);
  int getFilled();
  void setFilled(int value);
}
