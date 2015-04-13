package com.github.rosjava.challenge.uorc_publisher;

import orc.DigitalInput;
import orc.Orc;

import rss_msgs.DigitalStatusMsg;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

public class DigitalIOPublisher implements Runnable {

  ConnectedNode node;
  Orc orc;
  DigitalInput[] slowInputs = new DigitalInput[8];
  DigitalInput[] fastInputs = new DigitalInput[8];
  DigitalStatusMsg msg;
  Publisher<DigitalStatusMsg> pub;
  Object lock;

  public DigitalIOPublisher(ConnectedNode node, Orc orc, Object lock){
    this.node = node;
    this.orc = orc;
    this.lock = lock;
    for (int i = 0; i < 16; i++){
      if(i < 8){
        slowInputs[i] = new DigitalInput(orc, i, false, false); 
      } else {
        int j = i - 8;
        fastInputs[j] = new DigitalInput(orc, i, false, false);
      }
    }
    pub = node.newPublisher("rss/DigitalIO", "rss_msgs/DigitalStatusMsg");
  }

  @Override public void run() {
    msg = pub.newMessage();
    while (true){
      synchronized(lock) {
        boolean []fastVal = new boolean[8];
        boolean []slowVal = new boolean[8];
        for (int i = 0; i <8; i ++) {
          synchronized(lock) {
            fastVal[i] = fastInputs[i].getValue();
            slowVal[i] = slowInputs[i].getValue();
          }
        }
        msg.setFast(fastVal);
        msg.setSlow(slowVal);
        pub.publish(msg);
      }
      try {
        Thread.sleep(50);
      } catch (InterruptedException e){
        e.printStackTrace();
      }
    }
  }
}
