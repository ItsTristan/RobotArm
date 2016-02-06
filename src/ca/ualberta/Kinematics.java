package ca.ualberta;
import java.io.File;

import org.jblas.DoubleMatrix;
import org.jblas.Solve;

public class Kinematics {
	
	
	public static final int[] link_lengths = {135, 105};	// the length of each joint in mm.
	
	/**
	 * Given a set of theta values for the joint angles, returns
	 * the (x,y)-point location of the end effector
	 * @param theta
	 * @return
	 */
	public static Point3D forwardKinematics(int[] theta) {
		double thetaA = Math.toRadians(theta[0]);
		double thetaB = Math.toRadians(theta[1]);
		double x = (double) (link_lengths[0]*Math.cos(thetaA) + 
						link_lengths[1]*Math.cos(thetaA + thetaB));
		double y = (double) (link_lengths[0]*Math.sin(thetaA) + 
						link_lengths[1]*Math.sin(thetaA + thetaB));
		Point3D p = new Point3D(x,y);
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
	public static int getAngleBWlines(Point3D c, Point3D a, Point3D b){
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

	public static int[] inverseKinematics(Point3D target) {
		return inverseAnalyticKinematics(target);
	}

	public static int[] inverseKinematics(Point3D target, int[] theta0) {
		return inverseNumericalKinematics(target, theta0);
	}
	
	/**
	 * Given a point, computes and returns the joint angles
	 * required to achieve the target
	 * @param target
	 * @return final theta values
	 */
	public static int[] inverseAnalyticKinematics(Point3D target) {
		double[] angles = new double[2];
		double r = target.x*target.x + target.y*target.y;
		double l1 = link_lengths[0];
		double l2 = link_lengths[1];
		
		angles[1] = Math.acos((r-l1*l1-l2*l2) / (2*l1*l2));
		angles[0] = Math.asin(-l2*Math.sin(angles[1])/(Math.sqrt(r)))
										+ Math.atan2(target.y, target.x);
		
		int[] theta = new int[angles.length];
		for (int i = 0; i < theta.length; i++) {
			theta[i] = (int) Math.round(Math.toDegrees(angles[i]));
		}
		return theta;
	}
	
	/**
	 * Solves the inverse kinematics problem using
	 * Newton's method.
	 * @param target
	 * @param theta0
	 * @return
	 */
	public static int[] inverseNumericalKinematics(Point3D target, int[] theta0) {
		double[] theta = new double[theta0.length];
		int[] round = new int[theta0.length];
		
		// Copy theta0 to angles
		for (int j = 0; j < theta.length; j++)
			theta[j] = (double)Math.toRadians(theta0[j]);

		DoubleMatrix angles = new DoubleMatrix(theta);
		
		for (int i = 0; i < 100; i++) {
			// Get the error term
			round = angles.mul(180d/Math.PI).toIntArray();
			Point3D actual = forwardKinematics(round);
			DoubleMatrix error = new DoubleMatrix(
					new double[]{target.x - actual.x, target.y - actual.y});

			// If error is small enough, stop
			if (error.norm2() <= Math.sqrt(TestKinematics.dist_thresh)) {
				break;
			}
			
			// Solve for delta theta
			DoubleMatrix delta = getInverseNumericalStep(error, angles);
			
			// Update the angles
			angles = angles.add(delta);
			
			// Repeat a few times to try to reduce the error.
		}

		// Round the result
		return angles.mul(180d/Math.PI).toIntArray();
	}
	
	/**
	 * Gives one update step, d(theta), for the given target point and
	 * theta0 values
	 * @param error
	 * @param angles, in radians
	 * @return
	 */
	public static DoubleMatrix getInverseNumericalStep(DoubleMatrix error, DoubleMatrix angles) {
		// Compute the Jacobian
		// Convention: Jac[row][col]
		double[][] Jac = new double[2][2];
		// dx/d0
		Jac[0][0] = -link_lengths[0]*Math.sin(angles.get(0))
				-link_lengths[1]*Math.sin(angles.get(0) + angles.get(1));
		Jac[0][1] = -link_lengths[1]*Math.sin(angles.get(0) + angles.get(1));
		
		Jac[1][0] = link_lengths[0]*Math.cos(angles.get(0))
				+link_lengths[1]*Math.cos(angles.get(0) + angles.get(1));
		Jac[1][1] = link_lengths[1]*Math.cos(angles.get(0) + angles.get(1));
		
		DoubleMatrix J = new DoubleMatrix(Jac);
		
		// Solve for delta Theta
		return Solve.solve(J, error);
	}
	
	/**
	 * Gives one update step, d(theta), for the given target point and
	 * theta0 values
	 * @param error
	 * @param angles, in radians
	 * @return
	 */
	public static double[] getInverseNumericalStep(double[] error, double[] angles) {
		return getInverseNumericalStep(new DoubleMatrix(error), new DoubleMatrix(angles)).toArray();
	}
	
	/**
	 * Gives one update step, d(theta), for the given target point and
	 * theta0 values
	 * @param error
	 * @param angles, in degrees
	 * @return
	 */
	public static double[] getInverseNumericalStep(double[] error, int[] angles) {
		double[] thetas = new double[angles.length];
		for (int i = 0; i < thetas.length; i++)
			thetas[i] = Math.toRadians(angles[i]);
		return getInverseNumericalStep(new DoubleMatrix(error), new DoubleMatrix(thetas)).toArray();
	}
	
	/**
	 * Given two points, returns a list of target points for the robot
	 * to move towards along a line, or null if the target has been reached.
	 * @param start
	 * @param end
	 * @param resolution the number of intermediate points to generate 
	 * @return
	 */
	public static Point3D[] createLinePath(Point3D current, Point3D start, Point3D end, int resolution) {
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
	public static Point3D[] createArcPath(Point3D current, Point3D points[], int resolution) {
		return null;
	}
	
	/**
	 * Given a 2D image, returns a path that traces the image.
	 * @param current
	 * @return
	 */
	public static Point3D[] createPicturePath(Point3D current, File image) {
		return null;
	}
}
