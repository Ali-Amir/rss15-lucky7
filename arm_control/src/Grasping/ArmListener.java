package Grasping;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.ArmMsg;

public class ArmListener implements MessageListener<ArmMsg> {

	private GraspingFSM fsm;
	private Grasping grasp;
	private boolean isFSM;

	public ArmListener(GraspingFSM graspingFSM) {
		this.fsm = graspingFSM;
		isFSM = true;
	}

	public ArmListener(Grasping grasping) {
		this.grasp = grasping;
		isFSM = false;
	}

	@Override
	public void onNewMessage(ArmMsg msg) {
		if(isFSM){
			fsm.handle(msg);
		} else {
			grasp.handle(msg);
		}
	}
}


