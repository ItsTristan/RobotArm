package ca.ualberta;

import java.awt.Point;
import java.io.File;

public class Kinematics {
	
	
	public static final int[] link_lengths = {135, 105};	// the length of each joint in mm.
	
	/**
	 * Given a set of theta values for the joint angles, returns
	 * the (x,y)-point location of the end effector
	 * @param theta
	 * @return
	 */
	public static Point forwardKinematics(int[] theta) {
		double thetaA = Math.toRadians(theta[0])/2;
		double thetaB = Math.toRadians(theta[1]);
		int x = (int) (link_lengths[0]*Math.cos(thetaA) + 
						link_lengths[1]*Math.cos(thetaA + thetaB));
		int y = (int) (link_lengths[0]*Math.sin(thetaA) + 
						link_lengths[1]*Math.sin(thetaA + thetaB));
		Point p = new Point(x,y);
		return p;
	}
	
	/**
	 * Given a point, computes and returns the joint angles
	 * required to achieve the target
	 * @param target
	 * @return final theta values
	 */
	public static int[] inverseKinematics(Point target) {
		return null;
	}
	
	/**
	 * Given two points, returns a list of target points for the robot
	 * to move towards along a line, or null if the target has been reached.
	 * @param start
	 * @param end
	 * @param resolution the number of intermediate points to generate 
	 * @return
	 */
	public static Point[] createLinePath(Point current, Point start, Point end, int resolution) {
		return null;
	}
	
	/**
	 * Given n points, returns a list of target points for the robot to move
	 * towards along an arc, or null if the target has been reached.
	 * @param current
	 * @param start
	 * @param mid
	 * @param end
	 * @param resolution
	 * @return
	 */
	public static Point[] createArcPath(Point current, Point points[], int resolution) {
		return null;
	}
	
	/**
	 * Given a 2D image, returns a path that traces the image.
	 * @param current
	 * @return
	 */
	public static Point[] createPicturePath(Point current, File image) {
		return null;
	}
}
