package com.github.rosjava.challenge.uorc_publisher;

import orc.Orc;
import orc.QuadratureEncoder;

import rss_msgs.EncoderMsg;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

public class EncoderPublisher implements Runnable {

    ConnectedNode node;
    Orc orc;
    QuadratureEncoder left;
    QuadratureEncoder right;
    EncoderMsg msg;
    Publisher<EncoderMsg> pub;
    Object lock;
	
  public EncoderPublisher(ConnectedNode node, Orc orc, Object lock) {
    this.node = node;
    this.orc = orc;
    this.lock = lock;
    left = new QuadratureEncoder(orc, 0, false);
    right = new QuadratureEncoder(orc, 1, true);
    pub = node.newPublisher("rss/Encoder", "rss_msgs/EncoderMsg");
  }
    
  @Override public void run() {
    msg = pub.newMessage();
    while(true){
      synchronized(lock) {
        msg.setLeft(left.getPosition());
        msg.setRight(right.getPosition());
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
