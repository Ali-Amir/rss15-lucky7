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

package com.github.rosjava.challenge.gui;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import rss_msgs.MotionMsg;
import rss_msgs.OdometryMsg;


public class VisionGUI extends AbstractNodeMain {

	/**
	 * <p> Node for ROS communication </p>
	 */
	public ConnectedNode node;

	private Subscriber<OdometryMsg> odoSub;
	private Subscriber<sensor_msgs.Image> vidSub;

  public VisionGUIPanel panel;

	protected boolean firstUpdate = true;


	public VisionGUI(int poseSaveInterval, double maxTV, double maxRV) {
    panel = new VisionGUIPanel(poseSaveInterval, maxTV, maxRV);
  }

	public VisionGUI(int poseSaveInterval) {
		panel = new VisionGUIPanel(poseSaveInterval);
	}

	public VisionGUI() {
		panel = new VisionGUIPanel();
	}


	/**
	 * <p>See {@link #instanceMain}.</p>
	 **/
	@Override
	public void onStart(ConnectedNode node) {
		this.node = node;

		final boolean reverseRGB = node.getParameterTree().getBoolean("reverse_rgb", false);

		vidSub = node.newSubscriber("/rss/video", "sensor_msgs/Image");
		vidSub.addMessageListener(new MessageListener<sensor_msgs.Image>() {
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
				panel.setVisionImage(rgbData, (int)message.getWidth(), (int)message.getHeight());
			}
		}
		);

		odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
		odoSub.addMessageListener(
				new MessageListener<rss_msgs.OdometryMsg>() {
					@Override
					public void onNewMessage(rss_msgs.OdometryMsg message) {
						if ( firstUpdate ) {
							firstUpdate = false;
              //Robot.resetRobotBase();
              //Robot.setVelocity(0.0, 0.0);
							panel.resetWorldToView(message.getX(), message.getY());
						}
						panel.setRobotPose(message.getX(), message.getY(), message.getTheta());
					}
				}
		);

    panel.initPublisher(node);
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("rss/visiongui");
	}

}
