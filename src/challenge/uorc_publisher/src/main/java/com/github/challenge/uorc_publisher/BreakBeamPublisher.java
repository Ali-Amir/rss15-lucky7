package com.github.rosjava.challenge.uorc_publisher;

import orc.DigitalInput;
import orc.Orc;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import rss_msgs.*;

public class BreakBeamPublisher implements Runnable {
	
  ConnectedNode node;
  Orc orc;
  DigitalInput input;
  BreakBeamMsg msg;
  Publisher<BreakBeamMsg> pub;
  Object lock;

  public BreakBeamPublisher(ConnectedNode node, Orc orc, Object lock) {
    this.node = node;
    this.orc = orc;
    this.lock = lock;
    input = new DigitalInput(orc, 7, false, false);
    pub = node.newPublisher("rss/BreakBeam", BreakBeamMsg._TYPE);
  }

  @Override public void run() {
    // TODO Auto-generated method stub
    msg = pub.newMessage();
    while (true){
      synchronized(lock) {
        msg.setBeamBroken(input.getValue());
      }
      pub.publish(msg);
      try {
        Thread.sleep(50);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }
  }
    
}
