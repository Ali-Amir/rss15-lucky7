package com.github.rosjava.challenge.odometry;

import org.ros.message.MessageListener;
import rss_msgs.EncoderMsg;
import rss_msgs.OdometryMsg;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.AbstractNodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public class Odometry extends AbstractNodeMain {

	private int[] prev_ticks; // (left wheel, right wheel, make int[]
	private static final double WHEEL_RADIUS_IN_M = 0.0625;
	private static final double ENCODER_RESOLUTION = 2000;
	private static final double GEAR_RATIO = 65.5;
	private static final double TICKS_PER_REVOLUTION=ENCODER_RESOLUTION * GEAR_RATIO;
	private static final double WHEEL_METERS_PER_TICK = WHEEL_RADIUS_IN_M * Math.PI * 2/(TICKS_PER_REVOLUTION);
	private static final double WHEELBASE = .428;
	/**
	 * [x][y][theta]
	 */
	private double[] pose = {0, 0, 0};
	private boolean reset = false;
	private OdometryMsg msg;
	private Publisher<OdometryMsg> pub;
	private Publisher<std_msgs.String> statePub;
	private Subscriber<EncoderMsg> encoderSub;
	private Subscriber<OdometryMsg> resetOdometrySub;

	public void update(int[] new_ticks) {
		if ((prev_ticks == null && (new_ticks[0]==0 && new_ticks[1]==0)) || reset) {
			prev_ticks = new_ticks;
			reset = false;
		}
		else if (prev_ticks == null) {
			prev_ticks = new int[2];
		}
		int[] ticks = new int[2];
		for (int i = 0; i < 2; i ++){
			ticks[i] = new_ticks[i] - prev_ticks[i];
		}
		if (ticks[0] == 0 && ticks[1] == 0) {
			pub.publish(msg);
			return; // we haven't moved at all
		}

		double s_left = (ticks[0])*WHEEL_METERS_PER_TICK;
		double s_right = (ticks[1])*WHEEL_METERS_PER_TICK;
		double theta = (s_left - s_right)/WHEELBASE;

    double new_theta = (msg.getTheta()-theta) % (2*Math.PI);
    msg.setTheta(new_theta);
		msg.setX(msg.getX() + (s_left+s_right)*Math.cos(new_theta)/2.0);
		msg.setY(msg.getY() + (s_left+s_right)*Math.sin(new_theta)/2.0);

		prev_ticks[0] = new_ticks[0];
		prev_ticks[1] = new_ticks[1];
		pub.publish(msg);
	}

	@Override
	public void onStart(ConnectedNode node) {
		pub = node.newPublisher("/rss/odometry", OdometryMsg._TYPE);
    msg = pub.newMessage();

		encoderSub = node.newSubscriber("/rss/Encoder", EncoderMsg._TYPE);
		encoderSub.addMessageListener(new EcoderListener(this));

		resetOdometrySub = node.newSubscriber("/rss/odometry_update", OdometryMsg._TYPE);
		resetOdometrySub.addMessageListener(new MessageListener<rss_msgs.OdometryMsg>() {
			@Override
			public void onNewMessage(rss_msgs.OdometryMsg message) {
				if (message.getX() == 0 && message.getY() == 0 && message.getTheta() == 0) {
					prev_ticks = null;
					msg.setX(message.getX()); msg.setY(message.getY()); msg.setTheta(message.getTheta());
					reset = true;
				}
			}
		});
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("rss/odometry");
	}
}
