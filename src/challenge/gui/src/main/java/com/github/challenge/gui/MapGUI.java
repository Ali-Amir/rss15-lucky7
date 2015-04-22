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

import java.awt.*;
import java.awt.geom.*;
import javax.swing.*;

import org.ros.node.ConnectedNode;
import org.ros.node.AbstractNodeMain;
import org.ros.namespace.GraphName;
import org.ros.node.topic.Subscriber;

import java.util.*;


/**
 * <p>Extends <code>LocalNavigation.SonarGUI</code> to display map-related
 * data (first read the doc for that class).</p>
 *
 * <p>New methods (and corresponding ROS messages) have been added to draw
 * rectangles and polygons.</p>
 *   
 * @author vona
 **/
public class MapGUI extends SonarGUI {
  
  /**
   * <p>Consruct a new MapGUI.</p>
   *
   * <p>See <code>LocalNavigation.SonarGUI(int, double, double)</code>.</p>
   **/
  /*
  public MapGUI(int poseSaveInterval, double maxTV, double maxRV) {
    super();
    panel = new MapGUIPanel(poseSaveInterval, maxTV, maxRV);
  }
  */

  /**
   * <p>See <code>LocalNavigation.SonarGUI(int)</code>.</p>
   **/
  /*
  public MapGUI(int poseSaveInterval) {
    super();
    panel = new MapGUIPanel(poseSaveInterval);
  }
  */

  /**
   * <p>See <code>LocalNavigation.SonarGUI()</code>.</p>
   **/
  public MapGUI() {
    //super();
    //panel = new MapGUIPanel();
  }

  private Subscriber<gui_msgs.GUIRectMsg> guiRectSub;
  private Subscriber<gui_msgs.GUIPolyMsg> guiPolySub;
  private Subscriber<gui_msgs.GUIEraseMsg> guiEraseSub;

  /**
   * Hook called by ROS to start the gui
   **/
  public void onStart(ConnectedNode node) {
    if (panel == null) {
      panel = new MapGUIPanel();
    }

    guiRectSub = node.newSubscriber("gui/Rect", gui_msgs.GUIRectMsg._TYPE);
    guiRectSub.addMessageListener(new RectMessageListener(this), 1000);
    guiPolySub = node.newSubscriber("gui/Poly", gui_msgs.GUIPolyMsg._TYPE);
    guiPolySub.addMessageListener(new PolyMessageListener(this), 1000);
    guiEraseSub = node.newSubscriber("gui/Erase", gui_msgs.GUIEraseMsg._TYPE);
    guiEraseSub.addMessageListener(new MapEraseMessageListener(this));
    super.onStart(node);
  }
    
}
