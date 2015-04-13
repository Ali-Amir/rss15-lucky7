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

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.*;
import org.ros.node.ConnectedNode;

import org.ros.node.topic.Subscriber;


/**
 * <p>Extends <code>vision.VisionGUI</code> to display sonar-related data
 * (first read the doc for that class).</p>
 *
 * <p>New methods (and corresponding Carmen messages) have been added to draw
 * points, lines, and line segments.</p>
 *   
 * @author vona
 **/
public class SonarGUI extends VisionGUI {

  public Subscriber<gui_msgs.GUILineMsg> guiLineSub;
  public Subscriber<gui_msgs.GUISegmentMsg> guiSegmentSub;
  public Subscriber<gui_msgs.GUIPointMsg> guiPointSub;
  public Subscriber<gui_msgs.GUIEraseMsg> guiEraseSub;

  public SonarGUIPanel panel;

	public SonarGUI(int poseSaveInterval, double maxTV, double maxRV) {
    panel = new SonarGUIPanel(poseSaveInterval, maxTV, maxRV);
  }

	public SonarGUI(int poseSaveInterval) {
		panel = new SonarGUIPanel(poseSaveInterval);
	}

	public SonarGUI() {
		panel = new SonarGUIPanel();
	}

  /**
   * <p>See <code>vision.VisionGUI.instanceMain()</code> and {@link
   * #mainPostHook}.</p>
   **/
  @Override
  public void onStart(ConnectedNode node) {
    super.onStart(node);
    guiLineSub = node.newSubscriber("gui/Line", gui_msgs.GUILineMsg._TYPE);
    guiSegmentSub = node.newSubscriber("gui/Segment", gui_msgs.GUISegmentMsg._TYPE);
    guiPointSub = node.newSubscriber("gui/Point", gui_msgs.GUIPointMsg._TYPE);
    guiEraseSub = node.newSubscriber("gui/Erase", gui_msgs.GUIEraseMsg._TYPE);

    guiLineSub.addMessageListener(new LineMessageListener(this));
    guiSegmentSub.addMessageListener(new SegmentMessageListener(this));
    guiPointSub.addMessageListener(new PointMessageListener(this));
    guiEraseSub.addMessageListener(new SonarEraseMessageListener(this));
  }

}
