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
		double thetaA = Math.toRadians(theta[0]);
		double thetaB = Math.toRadians(theta[1]);
		int x = (int) (link_lengths[0]*Math.cos(thetaA) + 
						link_lengths[1]*Math.cos(thetaA + thetaB));
		int y = (int) (link_lengths[0]*Math.sin(thetaA) + 
						link_lengths[1]*Math.sin(thetaA + thetaB));
		Point p = new Point(x,y);
		return p;
	}
	
	/**
	 * Given 3 points of two intersecting lines find the angle they intersect at
	 * @param c intersecting point
	 * @param a point from first line
	 * @param b poit from other line
	 * @return angle between the two lines at point c
	 */
		//NEEDS DEBUGGING
	public static int getAngleBWlines(Point c, Point a, Point b){
		// get absolute side lengths
		// sqrt((x2-x1)^2 + (y2-y1)^2)
//		double AC = Math.sqrt((Math.pow(c.x - a.x, 2) +  Math.pow(c.y - a.y, 2)));
//		double CB = Math.sqrt((Math.pow(b.x - c.x, 2) +  Math.pow(b.y - c.y, 2)));
//		double AB = Math.sqrt((Math.pow(b.x - a.x, 2) +  Math.pow(b.y - a.y, 2)));
		
		double AB = a.distance(b);
		double CA = c.distance(a);
		double CB = c.distance(b);
		
		// law of cosines to find angle at point corner c
		// c^2 = a^2 + b^2 - 2abcos(C) where C is angle opposite AB edge at point c
		double cosAngle = (CA*CA +  CB*CB - AB*AB) / (2*CA*CB);
		int angle = (int)Math.toDegrees(Math.acos(cosAngle));
		return angle;
	}
	
	/**
	 * Given a point, computes and returns the joint angles
	 * required to achieve the target
	 * @param target
	 * @return final theta values
	 */
	public static int[] inverseAnalyticKinematics(Point target) {
		int[] angles = new int[2]; 
			double xsqr_ysqr = target.x*target.x + target.y*target.y;
			angles[1] = (int) Math.toDegrees(Math.acos( ( xsqr_ysqr - link_lengths[0]*link_lengths[0] - link_lengths[1]*link_lengths[1])
							/ (2*link_lengths[0]*link_lengths[1]) ));
			angles[0] = (int) Math.toDegrees(Math.asin((link_lengths[1]*Math.sin(angles[1]))
											/(Math.sqrt(xsqr_ysqr)))
											+ Math.atan2(target.y, target.x));
		return angles;
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
