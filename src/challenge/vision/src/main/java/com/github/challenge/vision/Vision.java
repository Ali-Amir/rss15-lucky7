/*
 * Copyright (C) 2014 Ali-Amir Aldan.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.github.rosjava.challenge.vision;

import java.util.concurrent.ArrayBlockingQueue;

import org.ros.message.MessageListener;
import rss_msgs.MotionMsg;
import rss_msgs.OdometryMsg;
import rss_msgs.RobotLocation;
import rss_msgs.LocFree;
import rss_msgs.LocFreeRequest;
import rss_msgs.LocFreeResponse;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import com.github.rosjava.challenge.gui.Image;
import com.github.rosjava.challenge.gui.VisionGUI;

/**
 * 
 * @author previous TA's, prentice, vona
 *
 */
public class Vision extends AbstractNodeMain implements Runnable {

	private static final int width = 160;

	private static final int height = 120;

  private double curLocX;
  private double curLocY;
  private double curLocTheta;
	private Publisher<sensor_msgs.Image> vidPub;


	/**
	 * <p>The blob tracker.</p>
	 **/
	private BlobTracking blobTrack = null;


	private VisionGUI gui;
	private ArrayBlockingQueue<byte[]> visionImage = new ArrayBlockingQueue<byte[]>(
			1);

	protected boolean firstUpdate = true;

	public Subscriber<RobotLocation> locSub;
	public Subscriber<sensor_msgs.Image> vidSub;
	public Subscriber<OdometryMsg> odoSub;
	
	public Publisher<MotionMsg> velocityPub;
	
	/**
	 * <p>Create a new Vision object.</p>
	 */
	public Vision() {

		setInitialParams();

		gui = new VisionGUI();
	}

	protected void setInitialParams() {

	}

	/**
	 * <p>Handle a CameraMessage. Perform blob tracking and
	 * servo robot towards target.</p>
	 * 
	 * @param rawImage a received camera message
	 */
	public void handle(byte[] rawImage) {
		visionImage.offer(rawImage);
	}

	@Override
	public void run() {
		while (true) {
			Image src = null;
			try {
				src = new Image(visionImage.take(), width, height);
			} catch (InterruptedException e) {
				e.printStackTrace();
				continue;
			}

			Image dest = new Image(src);

			blobTrack.apply(src, dest);

      sensor_msgs.Image pubImage = vidPub.newMessage();
      pubImage.setWidth(width);
      pubImage.setHeight(height);
      pubImage.setEncoding("rgb8");
      pubImage.setIsBigendian((byte)0);
      pubImage.setStep(width*3);
      pubImage.setData(org.jboss.netty.buffer.ChannelBuffers.
                        copiedBuffer(org.jboss.netty.buffer.ChannelBuffers.LITTLE_ENDIAN, dest.toArray()));
      vidPub.publish(pubImage);

			// update newly formed vision message
      //gui.panel.setVisionImage(dest.toArray(), width, height);

			// Begin Student Code
      /*
			double transVelocity = 0;
			double rotVelocity = 0;
      if (blobTrack.targetDetected) {
        blobTrack.blobFix();

        transVelocity = Math.min(0.2, (blobTrack.targetRange-0.5));
        transVelocity = Math.max(-0.2, transVelocity);

        rotVelocity = Math.min(Math.PI/20.0, blobTrack.targetBearing*0.2);
        rotVelocity = Math.max(-Math.PI/20.0, rotVelocity);
      }
      */

			// publish velocity messages to move the robot towards the target
      /*
      MotionMsg msg = velocityPub.newMessage();
			msg.setTranslationalVelocity(transVelocity);
			msg.setRotationalVelocity(rotVelocity);
			velocityPub.publish(msg);
      */

			// End Student Code
		}
	}

	/**
	 * <p>
	 * Run the Vision process
	 * </p>
	 * 
	 * @param node optional command-line argument containing hostname
	 */
	@Override
	public void onStart(final ConnectedNode node) {
    ServiceClient<LocFreeRequest, LocFreeResponse> client;
    while (true) {
      try {
        client = node.newServiceClient("/navigation/IsLocationFree", LocFree._TYPE);
        break;
      } catch (Exception e) {
        //System.err.println("onStart Vision: NO SERVICE islocationfree FOUND!!!!");
        //e.printStacktrace();
      }
    }
    System.out.println("Initialized Vision!");
		blobTrack = new BlobTracking(width, height, client);
    curLocX = 0.6;
    curLocY = 0.6;
    curLocTheta = 0.0;
    blobTrack.updateLocation(curLocX, curLocY, curLocTheta);
		vidPub = node.newPublisher("/rss/blobVideo", sensor_msgs.Image._TYPE);

		// initialize the ROS publication to command/Motors
		velocityPub = node.newPublisher("command/Motors", MotionMsg._TYPE);
		// End Student Code


		final boolean reverseRGB = node.getParameterTree().getBoolean("reverse_rgb", false);

		locSub = node.newSubscriber("/localization/update", RobotLocation._TYPE);
		locSub
		.addMessageListener(new MessageListener<RobotLocation>() {
			@Override
			public void onNewMessage(RobotLocation message) {
        curLocX = message.getX();
        curLocY = message.getY();
        curLocTheta = message.getTheta();
        blobTrack.updateLocation(curLocX, curLocY, curLocTheta);
			}
		});

		vidSub = node.newSubscriber("/rss/video", sensor_msgs.Image._TYPE);
		vidSub
		.addMessageListener(new MessageListener<sensor_msgs.Image>() {
			@Override
			public void onNewMessage(sensor_msgs.Image message) {
				byte[] rgbData;
				if (reverseRGB) {
					rgbData = Image.RGB2BGR(message.getData().array(),
							(int) message.getWidth(), (int) message.getHeight());
				}
				else {
					rgbData = message.getData().array();
				}
				assert ((int) message.getWidth() == width);
				assert ((int) message.getHeight() == height);
				handle(rgbData);
			}
		});

		odoSub = node.newSubscriber("/rss/odometry", OdometryMsg._TYPE);
		odoSub.addMessageListener(new MessageListener<OdometryMsg>() {
			@Override
			public void onNewMessage(OdometryMsg message) {
				if (firstUpdate) {
					firstUpdate = false;
					gui.panel.resetWorldToView(message.getX(), message.getY());
				}
				gui.panel.setRobotPose(message.getX(), message.getY(), message.getTheta());
			}
		});
		Thread runningStuff = new Thread(this);
		runningStuff.start();
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("rss/vision");
	}
}
