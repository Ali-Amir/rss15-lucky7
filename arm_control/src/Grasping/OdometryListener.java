package Grasping;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.OdometryMsg;

public class OdometryListener implements MessageListener<OdometryMsg> {

	private Grasping grasp;

	public OdometryListener(Grasping grasping) {
		this.grasp = grasping;
	}

	@Override
	public void onNewMessage(OdometryMsg msg) {
		grasp.handle(msg);
	}

}
