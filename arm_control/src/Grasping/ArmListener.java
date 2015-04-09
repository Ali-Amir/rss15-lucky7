package Grasping;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.ArmMsg;

public class ArmListener implements MessageListener<ArmMsg> {

	private Grasping grasp;

	public ArmListener(Grasping grasping) {
		this.grasp = grasping;
	}

	@Override
	public void onNewMessage(ArmMsg msg) {
    grasp.handle(msg);
	}
}


