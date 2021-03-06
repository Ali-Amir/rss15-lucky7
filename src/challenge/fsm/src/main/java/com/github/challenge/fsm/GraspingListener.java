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

package com.github.rosjava.challenge.fsm;

import org.ros.message.MessageListener;
import rss_msgs.GraspingMsg;

public class GraspingListener implements MessageListener<GraspingMsg> {

	private FSM object;

	public GraspingListener(FSM object) {
		this.object = object;
	}

	@Override
	public void onNewMessage(GraspingMsg msg) {
    object.handle(msg);
	}
}

