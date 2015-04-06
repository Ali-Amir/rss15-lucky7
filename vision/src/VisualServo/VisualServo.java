package VisualServo;

import java.util.concurrent.ArrayBlockingQueue;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

/**
 * 
 * @author previous TA's, prentice, vona
 *
 */
public class VisualServo implements NodeMain, Runnable {

	private static final int width = 160;

	private static final int height = 120;


	/**
	 * <p>The blob tracker.</p>
	 **/
	private BlobTracking blobTrack = null;


	private VisionGUI gui;
	private ArrayBlockingQueue<byte[]> visionImage = new ArrayBlockingQueue<byte[]>(
			1);

	protected boolean firstUpdate = true;

	public Subscriber<org.ros.message.sensor_msgs.Image> vidSub;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	
	public Publisher<org.ros.message.rss_msgs.MotionMsg> velocityPub;
	MotionMsg msg;
	
	/**
	 * <p>Create a new VisualServo object.</p>
	 */
	public VisualServo() {

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

			// update newly formed vision message
			gui.setVisionImage(dest.toArray(), width, height);

			// Begin Student Code
			double transVelocity = 0;
			double rotVelocity = 0;
      if (blobTrack.targetDetected) {
        blobTrack.blobFix();

        transVelocity = Math.min(0.2, (blobTrack.targetRange-0.5));
        transVelocity = Math.max(-0.2, transVelocity);

        rotVelocity = Math.min(Math.PI/20.0, blobTrack.targetBearing*0.2);
        rotVelocity = Math.max(-Math.PI/20.0, rotVelocity);
      }
			
			
			// publish velocity messages to move the robot towards the target
			msg.translationalVelocity = transVelocity;
			msg.rotationalVelocity = rotVelocity;
			velocityPub.publish(msg);
			
			
			// End Student Code
		}
	}

	/**
	 * <p>
	 * Run the VisualServo process
	 * </p>
	 * 
	 * @param node optional command-line argument containing hostname
	 */
	@Override
	public void onStart(Node node) {
		blobTrack = new BlobTracking(width, height);

		// Begin Student Code

		// set parameters on blobTrack as you desire



		// initialize the ROS publication to command/Motors
		velocityPub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
		msg = new MotionMsg();
		// End Student Code


		final boolean reverseRGB = node.newParameterTree().getBoolean("reverse_rgb", false);

		vidSub = node.newSubscriber("/rss/video", "sensor_msgs/Image");
		vidSub
		.addMessageListener(new MessageListener<org.ros.message.sensor_msgs.Image>() {
			@Override
			public void onNewMessage(
					org.ros.message.sensor_msgs.Image message) {
				byte[] rgbData;
				if (reverseRGB) {
					rgbData = Image.RGB2BGR(message.data,
							(int) message.width, (int) message.height);
				}
				else {
					rgbData = message.data;
				}
				assert ((int) message.width == width);
				assert ((int) message.height == height);
				handle(rgbData);
			}
		});

		odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
		odoSub
		.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
			@Override
			public void onNewMessage(
					org.ros.message.rss_msgs.OdometryMsg message) {
				if (firstUpdate) {
					firstUpdate = false;
					gui.resetWorldToView(message.x, message.y);
				}
				gui.setRobotPose(message.x, message.y, message.theta);
			}
		});
		Thread runningStuff = new Thread(this);
		runningStuff.start();
	}

	@Override
	public void onShutdown(Node node) {
		if (node != null) {
			node.shutdown();
		}
	}

	@Override
	public void onShutdownComplete(Node node) {
	}

	@Override
	public GraphName getDefaultNodeName() {
		return new GraphName("rss/visualservo");
	}
}
