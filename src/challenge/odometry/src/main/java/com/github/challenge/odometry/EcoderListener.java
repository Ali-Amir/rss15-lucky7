package com.github.rosjava.challenge.odometry;

import org.ros.message.MessageListener;
import rss_msgs.EncoderMsg;

public class EcoderListener implements MessageListener<EncoderMsg> {

	private Odometry parent;
	
	public EcoderListener(Odometry odometry) {
		parent = odometry;
	}

	@Override
	public void onNewMessage(EncoderMsg msg) {
		int[] ticks = new int[2];
		ticks[0] = (int) msg.getLeft();
		ticks[1] = (int) msg.getRight();
		parent.update(ticks);
	}

}
