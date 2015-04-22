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

/*
 * ArmPoseGUI.java
 *
 * @author edsinger
 * @author prentice
 */

import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.Box;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.JTextField;
import javax.swing.border.EtchedBorder;
import javax.swing.border.TitledBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import rss_msgs.ArmMsg;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.AbstractNodeMain;
import org.ros.node.topic.Publisher;

@SuppressWarnings("serial")
public class ArmPoseGUI extends AbstractNodeMain {

  ArmPoseGUIPanel panel;

  ArmPoseGUI() {
    panel = new ArmPoseGUIPanel();
  }

	@Override public GraphName getDefaultNodeName() {
		return GraphName.of("rss/armposegui");
	}

	@Override
	public void onStart(ConnectedNode node) {
    panel.initPub(node);
	}

  private class ArmPoseGUIPanel extends JFrame {
    private ChannelPanel channel0;
    private ChannelPanel channel1;
    private ChannelPanel channel2;
    private ChannelPanel channel3;

    private long[] armTheta=new long[]{0,0,0,0,0,0,0,0};

    private ArmVisualizerPanel avPanel;

    private Publisher<ArmMsg> armPub;


    /**
     * Constructor for ArmPoseGUI
     */
    public ArmPoseGUIPanel() {
      channel0 = new ChannelPanel("Shoulder",0);
      channel1 = new ChannelPanel("Wrist",1);
      channel2 = new ChannelPanel("Gripper",2);
      channel3 = new ChannelPanel("N/A",3);
      addArmVisualizerPanel();

      // Adding components to container
      Container cp = getContentPane();
      cp.setLayout(new BorderLayout());
      Box top = Box.createHorizontalBox();
      top.add(channel0);
      top.add(Box.createHorizontalStrut(5));
      top.add(channel1);
      top.add(Box.createHorizontalStrut(5));
      top.add(channel2);
      top.add(Box.createHorizontalStrut(5));
      top.add(channel3);
      top.add(Box.createVerticalStrut(10));
      top.add(avPanel);
      cp.add(top, BorderLayout.CENTER);
      setDefaultCloseOperation(EXIT_ON_CLOSE);
      pack();
      setTitle( "ArmPose" );
      setSize(1200,700);
      setVisible( true );
    }

    public void initPub(ConnectedNode node) {
      armPub = node.newPublisher("command/Arm", ArmMsg._TYPE);
    }

    private class ChannelPanel extends JPanel{
      private JLabel desiredLabel;
      private JSlider desiredSlider;
      private JTextField desiredValue;
      private Box displayBox;
      private int channel;
      private ChannelPanelListener cpListener;

      public ChannelPanel(String title, int channel){
        super(new BorderLayout());
        this.channel = channel;
        cpListener = new ChannelPanelListener();
        displayBox = Box.createVerticalBox();
        displayBox.setPreferredSize(new java.awt.Dimension(500,20));
        displayBox.add(Box.createVerticalStrut(10));
        addSliderPanel();
        setBorder(new TitledBorder(new EtchedBorder(),title));
        add(displayBox, BorderLayout.CENTER );
      }

      private void addSliderPanel(){
        JPanel sliderPanel = new JPanel(new FlowLayout(FlowLayout.LEADING));

        desiredSlider = new JSlider(0,3000,1500);
        desiredSlider.setMajorTickSpacing(100);
        desiredSlider.setMinorTickSpacing(10);

        desiredSlider.setPaintTicks(false);//display tick marks
        desiredSlider.setPaintLabels(false);//display numbers
        desiredValue = new JTextField("0", 5);////<<---add a listener
        desiredValue.addActionListener(new ChannelPanelListener());
        desiredSlider.addChangeListener( new DesiredSliderListener(channel,desiredValue) );
        desiredValue.addActionListener(cpListener);
        desiredLabel = new JLabel("RawPwm");
        sliderPanel.add(desiredLabel);
        sliderPanel.add(desiredSlider);
        sliderPanel.add(desiredValue);
        displayBox.add(sliderPanel);
        displayBox.add(Box.createVerticalStrut(5));
      }

      private class ChannelPanelListener implements ActionListener{
        public void actionPerformed(ActionEvent e){
          desiredSlider.setValue(Integer.parseInt(desiredValue.getText()));
        }
      }

      private class DesiredSliderListener implements ChangeListener{
        private int channel;
        private JTextField label;
        public DesiredSliderListener(int c, JTextField j){
          channel=c;
          label =j;
        }
        public void stateChanged(ChangeEvent e) {
          JSlider source = (JSlider)e.getSource();
          label.setText(Integer.toString(source.getValue()));
          long val = source.getValue();

          System.out.println("Channel: " + channel + " PWM: " + val);

          armTheta[channel]=val;

          // send pose to Orc
          ArmMsg msg = armPub.newMessage();
          msg.setPwms(armTheta);
          armPub.publish(msg);
        }
      }
    }

    private void addArmVisualizerPanel(){
      avPanel = new ArmVisualizerPanel();
    }
  }

}




