package Grasping;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.BumpMsg;

public class BumpListener implements MessageListener<BumpMsg> {
	
	private Grasping grasp;

	public BumpListener(Grasping grasping) {
		this.grasp = grasping;
	}

	@Override
	public void onNewMessage(BumpMsg msg) {
    grasp.setBumpPressed(msg.gripper);
	}

}
