package rss_msgs;

public interface BumpMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rss_msgs/BumpMsg";
  static final java.lang.String _DEFINITION = "bool left\nbool right\nbool gripper\n";
  boolean getLeft();
  void setLeft(boolean value);
  boolean getRight();
  void setRight(boolean value);
  boolean getGripper();
  void setGripper(boolean value);
}
