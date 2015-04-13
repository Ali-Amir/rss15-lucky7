package com.github.rosjava.challenge.uorc_publisher;

import orc.DigitalInput;
import orc.Orc;

import rss_msgs.BumpMsg;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

public class BumperPublisher implements Runnable {

	public static final String BUMP_MSG = "rss_msgs/BumpMsg";
	public static final String BUMP_CHANNEL = "rss/BumpSensors";

	private DigitalInput left;
	private DigitalInput right;
	private DigitalInput gripper;
	private BumpMsg msg;
	private Publisher<BumpMsg> pub;
	private Object lock;

	public BumperPublisher(ConnectedNode node, Orc orc, Object lock){
		this.lock = lock;
		left = new DigitalInput(orc, 0, true, true);
		right = new DigitalInput(orc, 1, true, true);
		gripper = new DigitalInput(orc, 2, true, true);
		pub = node.newPublisher(BUMP_CHANNEL, BUMP_MSG);
	}

	@Override public void run() {
		msg = pub.newMessage();
		while(true){
			synchronized(lock) {
				msg.setLeft(left.getValue());
				msg.setRight(right.getValue());
				msg.setGripper(gripper.getValue());
			}
			pub.publish(msg);
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
}
