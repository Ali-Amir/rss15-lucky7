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
import gui_msgs.GUIPolyMsg;

import java.awt.Color;
import java.awt.geom.*;
import java.util.ArrayList;
import java.util.List;

public class PolyMessageListener implements MessageListener<GUIPolyMsg> {

	private MapGUI gui;

	public PolyMessageListener(MapGUI mapGUI) {
		gui = mapGUI;
	}

	@Override
	public synchronized void onNewMessage(GUIPolyMsg msg) {
		List<Point2D.Double> vertices = new ArrayList<Point2D.Double>();
		for (int i = 0; i < msg.getNumVertices(); i++){
			Point2D.Double p = new Point2D.Double(msg.getX()[i], msg.getY()[i]);
			vertices.add(p);
		}
		boolean closed = msg.getClosed() == 1;
		boolean filled = msg.getFilled() == 1;
		Color c = new Color((int)msg.getC().getR(), (int)msg.getC().getG(), (int)msg.getC().getB()); //gui.makeRandomColor();
    while (gui.panel == null);
    gui.panel.addPoly(vertices, closed, filled, c);
	}

}
