package com.github.rosjava.challenge.uorc_publisher;

import orc.Orc;
import orc.SRF02;

import rss_msgs.SonarMsg;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

public class SonarPublisher implements Runnable {
	ConnectedNode node;
	Orc orc;
	SRF02 sonar;
	boolean isFront;
	Publisher<SonarMsg> pub;
	final int frontAddr = 0x70;
	final int backAddr = 0x72;
	Object lock;

	public static void main(String[] args){
		Orc orc = Orc.makeOrc();
		SRF02 one = new SRF02(orc);
		SRF02 two = new SRF02(orc, 0x72);
		while(true){
			System.out.println("distance for one:\t" + one.measure());
			System.out.println("distance for two:\t" + two.measure());
		}
	}

	public SonarPublisher(ConnectedNode n, Orc o, boolean isFront, Object lock){
		this.node = n;
		this.orc = o;
		this.isFront = isFront;
		this.lock = lock;
		if (isFront){
			sonar = new SRF02(orc, frontAddr);
			pub = node.newPublisher("rss/Sonars/Front", "rss_msgs/SonarMsg");
			System.out.println("created front sonar publisher");
		} else {
			sonar = new SRF02(orc, backAddr);
			pub = node.newPublisher("rss/Sonars/Back", "rss_msgs/SonarMsg");
			System.out.println("created back sonar publisher");
		}
	}

	@Override public void run() {
		// TODO Auto-generated method stub
		SonarMsg msg = pub.newMessage();
		while(true){
			double s = 0.0;
			synchronized(lock) {
				s = sonar.measure();
			}
			if (s!=0){
				msg.setRange(s);
			}
			msg.setIsFront(isFront);
			pub.publish(msg);
		}
	}
}
