package gui_msgs;

public interface GUIPolyMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gui_msgs/GUIPolyMsg";
  static final java.lang.String _DEFINITION = "ColorMsg c\nint32 numVertices\nfloat32[] x\nfloat32[] y\nint32 closed\nint32 filled\n";
  gui_msgs.ColorMsg getC();
  void setC(gui_msgs.ColorMsg value);
  int getNumVertices();
  void setNumVertices(int value);
  float[] getX();
  void setX(float[] value);
  float[] getY();
  void setY(float[] value);
  int getClosed();
  void setClosed(int value);
  int getFilled();
  void setFilled(int value);
}
