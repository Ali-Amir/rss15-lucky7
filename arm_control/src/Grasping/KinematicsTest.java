package Grasping;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Point;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.geom.NoninvertibleTransformException;
import java.awt.geom.Point2D;

import javax.swing.JFrame;


@SuppressWarnings("serial")
public class KinematicsTest extends JFrame {
	
//	private KinematicsPanel kinematicsPanel;
	private KinematicsPanel kinematicsPanel;
	//private InverseKinematicsIterator iterator;
	
	public KinematicsTest () {
		kinematicsPanel = new KinematicsPanel();
		
		initComponents();
	}
	
	private void initComponents () {
		setLayout(new BorderLayout());
		

		getContentPane().add(kinematicsPanel, BorderLayout.CENTER);

		setSize(new Dimension(800, 600));
		validate();
		
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		setTitle("Kinematics Test");
		setVisible(true);
	}
	
	public static void main (String[] args) {
		javax.swing.SwingUtilities.invokeLater(new Runnable() {
	        public void run() {
	        	//Robot.initialize("KinematicsTest", "192.168.1.207");
	        	new KinematicsTest();      
	        }
		});
	}
	
	private class KinematicsPanel extends ArmVisualizerPanel {
		private Point2D.Double worldTarget = new Point2D.Double();
		private Point pixelTarget = new Point(); // pixel 
		
		public KinematicsPanel () {
			super();

			addMouseListener(new MouseListener () {
				public void mouseClicked(MouseEvent arg0) {}
				public void mousePressed(MouseEvent arg0) {
					pixelTarget = new Point(arg0.getX(), arg0.getY());
					executeKinematics();
					repaint();
				}
				public void mouseReleased(MouseEvent arg0) {}
				public void mouseEntered(MouseEvent arg0) {}
				public void mouseExited(MouseEvent arg0) {}
			});
			
			addMouseMotionListener(new MouseMotionListener (){
				public void mouseDragged(MouseEvent arg0) {
					pixelTarget = new Point(arg0.getX(), arg0.getY());
					executeKinematics();
					repaint();
				}
				public void mouseMoved(MouseEvent arg0) {}			
			});
		}
		
		@Override
		public void paint (Graphics g) {
			super.paint(g);
			
			g.setColor(Color.RED);
			g.drawOval(pixelTarget.x-5, pixelTarget.y-5, 10, 10);
		}
		
		private void executeKinematics () {
			try {			
				//stores in world coordinates where 
				worldToPixelTransform.inverseTransform(pixelTarget, worldTarget);
				System.out.println("Clicked at: ("+worldTarget.x+", "+worldTarget.y+")");
				
				//what is the new orientation for the arm (in radians)?
				
				// --- BEGIN STUDENT CODE --- //
				double new_shoulder_orientation = 0.0; // use inverse kinematics to store the orientation (in radians) of the shoulder
				double new_wrist_orientation = 0.0; // use inverse kinematics to store the orientation (in radians) of the wrist
				 // (Solution)
				double[] orientation = moveArmToXZ(worldTarget.x, worldTarget.y, ArmKinematics.ARM_LINK_1, ArmKinematics.ARM_LINK_2); // (Solution)
				new_shoulder_orientation = orientation[0]; // (Solution)
				new_wrist_orientation = orientation[1]; // (Solution)
				// --- END STUDENT CODE --- //
				
				kinematicsPanel.setShoulderOrientation(new_shoulder_orientation);
				kinematicsPanel.setWristOrientation(new_wrist_orientation);
			} catch (NoninvertibleTransformException e) {
				
			}
		}

		private double[] controlToTheta12(double pos1, double neg1, double pos2, int choices){ // (Solution)
			double theta1, theta2; // (Solution)
			 // (Solution)
			switch(choices){ // (Solution)
			case 1: // (Solution)
				theta2 = pos2; // (Solution)
				theta1 = pos1; // (Solution)
				break; // (Solution)
			case -1: // (Solution)
				theta2 = -pos2; // (Solution)
				theta1 = neg1; // (Solution)
				break; // (Solution)
			case 2: // (Solution)
				//can go either way // (Solution)
				//for now, just pick the + one // (Solution)
				//later, make it smarter?? // (Solution)
				System.out.println("IN CASE 2"); // (Solution)
				theta2 = pos2; // (Solution)
				theta1 = pos1; // (Solution)
				break; // (Solution)
			default: // (Solution)
				theta1 = Math.PI/2; // (Solution)
				theta2 = -Math.PI/2; // (Solution)
				break; // (Solution)
			} // (Solution)
			System.err.println("Choices: " + choices + " Theta 1: " + theta1 + ", Theta 2: " + theta2); // (Solution)
			double[] a = {theta1, theta2}; // (Solution)
			return a; // (Solution)
		} // (Solution)
		
		public double[] moveArmToXZ(double x, double z, double l1, double l2){ // (Solution)
			if(z < 0){ // (Solution)
				return null; // (Solution)
			} // (Solution)
			double xg = x-0.065; // (Solution)
			double zg = z-0.28; // (Solution)
			double r = Math.sqrt(Math.pow(xg, 2) + Math.pow(zg, 2)); // (Solution)
			if((r > (l1 + l2))|| // (Solution)
					r < (l1-l2)){ // (Solution)
//				armCommandError("radial distance"); // (Solution)
				return null; // (Solution)
			} //so now if all joints could rotate 180 degrees, we could get there // (Solution)
			
			double theta2pos = Math.acos((Math.pow(r,2) // (Solution)
					-Math.pow(l1,2) // (Solution)
					-Math.pow(l2,2)) // (Solution)
					/(2*l1*l2)); // (Solution)

			int choices = possibletheta2Choices(theta2pos, xg,zg,r, l1, l2); // (Solution)
			if(choices == 0){ // (Solution)
//				armCommandError("joint angle constraint"); // (Solution)
				return null; // (Solution)
			} // (Solution)
			double theta1pos = Math.atan2(zg, xg) - Math.asin(l2/r*Math.sin(theta2pos)); // (Solution)
			double theta1neg = Math.atan2(zg,xg)-Math.asin(l2/r*Math.sin(-theta2pos)); // (Solution)
			
			return controlToTheta12(theta1pos, theta1neg, theta2pos, choices); // (Solution)	
		} // (Solution)

		//Assumes: x,z is reachable with 360 degree possible joint rotations // (Solution)
		//returns 0 if commanded position is not possible // (Solution)
		//returns 1 if we must use the +theta2 solution // (Solution)
		//returns -1 if we must use the -theta2 solution // (Solution)
		//returns 2 if either theta2 will allow reaching x,y // (Solution)
		private int possibletheta2Choices(double theta2pos, double x, double z, double r, double l1, double l2){ // (Solution)
			double pos = theta2pos; // (Solution)
			double neg = -theta2pos; // (Solution)
			int ans = 3; // (Solution)
			double MIN_WRIST_ANGLE = -3*Math.PI/4; // (Solution)
			double MAX_WRIST_ANGLE = 3*Math.PI/4; // (Solution)
			double MIN_SHOULDER_ANGLE = -4*Math.PI/8; // (Solution)
			double MAX_SHOULDER_ANGLE = 7*Math.PI/8; // (Solution)
			if(!inRange(pos,MIN_WRIST_ANGLE,MAX_WRIST_ANGLE) // (Solution)
					&&!inRange(neg,MIN_WRIST_ANGLE,MAX_WRIST_ANGLE)){ // (Solution)
				return 0;//neither solution is in possible range // (Solution)
			} // (Solution)
			if(inRange(pos,MIN_WRIST_ANGLE,MAX_WRIST_ANGLE) // (Solution)
					&&!inRange(neg,MIN_WRIST_ANGLE,MAX_WRIST_ANGLE)){ // (Solution)
				ans = 1;	//still must see if shoulder solution is OK // (Solution)
			} // (Solution)
			else if(!inRange(pos,MIN_WRIST_ANGLE,MAX_WRIST_ANGLE) // (Solution)
					&& inRange(neg,MIN_WRIST_ANGLE,MAX_WRIST_ANGLE)){ // (Solution)
				ans = -1;	//still must see if shoulder solution OK // (Solution)
			} // (Solution)
			//calculate shoulder angle for each theta2 solution // (Solution)
			double theta1pos = Math.atan2(z, x) - Math.asin(l2/r*Math.sin(pos)); // (Solution)
			double theta1neg = Math.atan2(z,x)-Math.asin(l2/r*Math.sin(neg)); // (Solution)
			
			if(ans == 1){ // (Solution)
				if(inRange(theta1pos,MIN_SHOULDER_ANGLE,MAX_SHOULDER_ANGLE)) // (Solution)
					return ans; // (Solution)
				else // (Solution)
					return 0; // (Solution)
			} // (Solution)
			if(ans == -1){ // (Solution)
				if(inRange(theta1neg, MIN_SHOULDER_ANGLE, MAX_SHOULDER_ANGLE)) // (Solution)
					return ans; // (Solution)
				else // (Solution)
					return 0; // (Solution)
			} // (Solution)
			//two theta2's are possible for wrist.  Are they OK for shoulder? // (Solution)
			if(!inRange(theta1pos, MIN_SHOULDER_ANGLE, MAX_SHOULDER_ANGLE) // (Solution)
					&& !inRange(theta1neg, MIN_SHOULDER_ANGLE, MAX_SHOULDER_ANGLE)){ // (Solution)
				return 0; // (Solution)
			} // (Solution)

			if(inRange(theta1pos, MIN_SHOULDER_ANGLE, MAX_SHOULDER_ANGLE) // (Solution)
					&& !inRange(theta1neg, MIN_SHOULDER_ANGLE, MAX_SHOULDER_ANGLE)){ // (Solution)
				return 1; // (Solution)
			} // (Solution)
			if(!inRange(theta1pos, MIN_SHOULDER_ANGLE, MAX_SHOULDER_ANGLE) // (Solution)
					&& inRange(theta1neg, MIN_SHOULDER_ANGLE, MAX_SHOULDER_ANGLE)){ // (Solution)
				return -1; // (Solution)
			} // (Solution)
			//both choices are OK for shoulder // (Solution)
			return 2; // (Solution)
		}	 // (Solution)
		private boolean inRange(double num, double bottom, double top) // (Solution)
		{ // (Solution)
			if(num > bottom && num < top) // (Solution)
				return true; // (Solution)
			else // (Solution)
				return false; // (Solution)
		}	 // (Solution)
	}
}
