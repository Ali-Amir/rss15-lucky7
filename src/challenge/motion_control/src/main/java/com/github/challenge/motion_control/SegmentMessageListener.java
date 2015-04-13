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

package com.github.rosjava.challenge.motion_control;

import java.awt.Color;

import org.ros.message.MessageListener;
import gui_msgs.GUISegmentMsg;

public class SegmentMessageListener implements MessageListener<GUISegmentMsg> {

	private SonarGUI gui;

	public SegmentMessageListener(SonarGUI sonarGUI) {
		this.gui = sonarGUI;
	}

	@Override
	public void onNewMessage(GUISegmentMsg msg) {
		int r = (int) msg.getColor().getR();
		int g = (int) msg.getColor().getG();
		int b = (int) msg.getColor().getB();
		if (r<0 || g < 0 || b < 0){
			gui.panel.addSegment(msg.getStartX(), msg.getStartY(), msg.getEndX(), msg.getEndY());
		} else{
			Color c = new Color(r, g, b);
			gui.panel.addSegment(msg.getStartX(), msg.getStartY(), msg.getEndX(), msg.getEndY(), c);
		}
	}

}
