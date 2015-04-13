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

package com.github.rosjava.challenge.navigation;

import java.awt.Color;

import org.ros.message.MessageListener;
import gui_msgs.ColorMsg;
import gui_msgs.GUIRectMsg;

public class RectMessageListener implements MessageListener<GUIRectMsg> {

	private MapGUI gui;

	public RectMessageListener(MapGUI mapGUI) {
		this.gui = mapGUI;
	}

	@Override
	public void onNewMessage(GUIRectMsg message) {
		boolean filled = message.getFilled() == 1;
		Color color = getColorFromMsg(message.getC());
		gui.panel.addRect(message.getX(), message.getY(), message.getWidth(), message.getHeight(),
	              filled, color);
		
	}

	public Color getColorFromMsg(ColorMsg c) {
		Color color;
		if (c== null){
			color = gui.panel.rectColor;
		}else {
			color = new Color((int)c.getR(), (int)c.getG(), (int)c.getB());
		}
		return color;
	}

}
