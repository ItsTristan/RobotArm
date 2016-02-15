package ca.ualberta;

import java.io.File;

import lejos.hardware.Button;
import lejos.utility.Delay;
import lejos.utility.Matrix;

public class Kinematics {

	public static int step_size = 10;
	public static final int[] link_lengths = {135, 105, 85}; 	// the length of each joint in mm.

	/**
	 * Given a set of theta values for the joint angles, returns the (x,y)-point
	 * location of the end effector
	 * 
	 * @param theta
	 * @return
	 */
	public static Point3D forwardKinematics(int[] theta) {
		double thetaA = Math.toRadians(theta[0]);
		double thetaB = Math.toRadians(theta[1]);
		double thetaC = 0;
		if (theta.length > 2) {
			thetaC = Math.toRadians(theta[2]);
		}

		// == Base Parameters
		Matrix TBase = translate(0,0,link_lengths[2]);
		Matrix RBase = Matrix.identity(4, 4);

		// == Motor A Parameters
		Matrix TA = translate(link_lengths[0],0,0);
		Matrix RA = rotZ(thetaA);
		// == Motor B Parameters
		Matrix TB = translate(link_lengths[1],0,0);
		Matrix RB = rotZ(thetaB);
		// == Motor C Parameters
		Matrix TC = translate(0,0,-link_lengths[2]);
		Matrix RC = rotX(thetaC);

		Matrix end = RBase.times(TBase).times(RA).times(TA).times(RB).times(TB)
				.times(RC).times(TC)
				.times(new Matrix(new double[] { 0, 0, 0, 1 }, 4));

		return new Point3D(end);
	}

	/**
	 * Given 3 points of two intersecting lines find the angle they intersect at
	 * 
	 * @param c
	 *            intersecting point
	 * @param a
	 *            point from first line
	 * @param b
	 *            poit from other line
	 * @return angle between the two lines at point c
	 */
	public static int getAngleBWlines(Point3D c, Point3D a, Point3D b) {
		// get absolute side lengths
		double AB = a.distance(b);
		double CA = c.distance(a);
		double CB = c.distance(b);

		// law of cosines to find angle at point corner c
		// c^2 = a^2 + b^2 - 2abcos(C) where C is angle opposite AB edge at
		// point c
		double cosAngle = (CA * CA + CB * CB - AB * AB) / (2 * CA * CB);
		int angle = (int) Math.toDegrees(Math.acos(cosAngle));
		return angle;
	}

	/**
	 * finds center point of a circle given only 3 points on 
	 * that circle(so any arc with end points a and c)
	 * 
	 * @param a
	 *            start point of arc
	 * @param b
	 *            a point on the arc between a and b
	 * @param c
	 *            end point of arc
	 * @return center of circle
	 */
	// see math reference: http://www.regentsprep.org/regents/math/geometry/gcg6/RCir.htm
	public static Point3D getCenterArc(Point3D a, Point3D b, Point3D c){
		double slope_ab = 0;
		double slope_bc = 0;
		if (b.x != a.x){
			slope_ab = ( (a.y - b.y) / (a.x - b.x) );
		}
		if (c.x != b.x){
			slope_bc = ( (b.y - c.y) / (b.x - c.x) );
		}
		// line_y1= slope_ab*(x-a.x) + a.y
		// line_y2= slope_bc*(x-b.x) + b.y
		// intersection of these lines is at circle center
		double midx1 = (b.x + a.x)/2;
		double midy1 = (b.y + a.y)/2;
		double midx2 = (c.x + b.x)/2;
		double midy2 = (c.y + b.y)/2;
		double m1_slope = -1/slope_ab;
		double m2_slope = -1/slope_bc;
		//mid1_slope*(x-midx1) + midy1 = mid2_slope*(x-midx2) + midy2 solve for x
		double centerx = ( (m1_slope*midx1 - m2_slope*midx2 - midy1 + midy2) 
							/ (m1_slope - m2_slope) );
		double centery = m1_slope*(centerx-midx1) + midy1;
		Point3D center = new Point3D(centerx, centery);

		return center;
	}

	/**
	 * Automatically selects an appropriate inverse kinematics function
	 * given the number of theta values given.
	 * @param target The point to move the arm to
	 * @param theta0 the initial theta guess. If the length is 2, the analytic
	 * 	solution is used. Otherwise, the numerical solution is used. This
	 * 	should be length 2 or 3.
	 * @return the joint angles required to reach target.
	 */
	public static int[] inverseKinematics(Point3D target, int[] theta0) {
		if (theta0.length == 2) {
			return inverseAnalyticKinematics2D(target);
		} else {
			return inverseNumericalKinematics(target, theta0);
		}
	}

	/**
	 * Given a point, computes and returns the joint angles required to achieve
	 * the target
	 * 
	 * @param target
	 * @return final theta values
	 */
	public static int[] inverseAnalyticKinematics2D(Point3D target) {
		double[] angles = new double[3];
		double r = target.x * target.x + target.y * target.y;
		double l1 = link_lengths[0];
		double l2 = link_lengths[1];
		
		// if absolute value of r-l1*l1-l2*l2 is greater than one---> NAN
		// so only valid for points in: 30 < sqrt(x^2+y^2) < 240
		// Invalid for 3D inverse kinematics, so zero out the last entry
		angles[2] = 0;
		angles[1] = Math.acos((r - l1 * l1 - l2 * l2) / (2 * l1 * l2));
		angles[0] = Math.asin(-l2 * Math.sin(angles[1]) / (Math.sqrt(r)))
				+ Math.atan2(target.y, target.x);

		int[] theta = new int[angles.length];
		for (int i = 0; i < theta.length; i++) {
			theta[i] = (int) Math.round(Math.toDegrees(angles[i]));
		}
		// System.out.format("theta[1]= %d \ntheta[0]= %d\n", theta[1],
		// theta[0]);
		
		return theta;
	}

	/**
	 * Solves the inverse kinematics problem using Newton's method for
	 * the 2D problem.
	 * 
	 * @param target point to move to
	 * @param theta0 an initial guess close to the real solution.
	 * @return The joint angle required that gets to the target.
	 */
	public static int[] inverseNumericalKinematics2D(Point3D target,
			int[] theta0) {
		double[] theta = new double[2];
		int[] round = new int[3];
		round[2] = 0;

		// Copy theta0 to angles
		for (int j = 0; j < theta.length; j++)
			theta[j] = (double) Math.toRadians(theta0[j]);

		Matrix angles = new Matrix(theta, theta.length);

		for (int i = 0; i < 100; i++) {
			// Get the error term
			round = toIntArray(angles.times(180d / Math.PI));
			Point3D actual = forwardKinematics(round);
			Matrix error = new Matrix(new double[] { target.x - actual.x,
					target.y - actual.y}, 2);

			// If error is small enough, stop
			if (error.normF() <= Math.sqrt(TestKinematics.dist_thresh)) {
				break;
			}
			
			// Solve for delta theta
			Matrix delta = getInverseNumerical2DStep(error, angles);

			// Update the angles
			angles = angles.plus(delta);

			// Repeat a few times to try to reduce the error.
		}

		// Round the result
		return toIntArray(angles.times(180d / Math.PI));
	}

	/**
	 * Solves the inverse kinematics problem for 3 DOF. Uses a combination
	 * of Newton's method and gradient descent where appropriate.
	 * 
	 * @param target the point to try to move to
	 * @param theta0 joint angles that are close to the solution
	 * @return an int array of 3 joint angles to rotate to.
	 */
	public static int[] inverseNumericalKinematics(Point3D target, int[] theta0) {
		double thetaA = Math.toRadians(theta0[0]);
		double thetaB = Math.toRadians(theta0[1]);
		double thetaC = Math.toRadians(theta0[2]);

		Matrix theta = new Matrix(new double[] { thetaA, thetaB, thetaC }, 3);
		
		if (target.x == 0) {
			target = new Point3D(target.x+Math.random(), target.y, target.z);
		}
		if (target.y == 0) {
			target = new Point3D(target.x, target.y+Math.random(), target.z);
		}
		if (target.z == 0) {
			target = new Point3D(target.x, target.y, target.z+Math.random());
		}
		
		for (int i = 0; i < 100; i++) {
			// Error term
			Point3D actual = forwardKinematics(toIntArray(theta.times(180 / Math.PI)));
			
			Matrix dY = target.toMatrix().minus(actual.toMatrix());

			if (dY.normF() < (TestKinematics.dist_thresh))
				break;

			// == Base Parameters
			Matrix TBase = translate(0,0,link_lengths[2]);
			Matrix RBase = Matrix.identity(4, 4);

			// == Motor A Parameters
			Matrix TA = translate(link_lengths[0],0,0);
			Matrix RA = rotZ(thetaA);
			Matrix dRA = drotZ(thetaA);
			// == Motor B Parameters
			Matrix TB = translate(link_lengths[1],0,0);
			Matrix RB = rotZ(thetaB);
			Matrix dRB = drotZ(thetaB);
			// == Motor C Parameters
			Matrix TC = translate(0,0,-link_lengths[2]);
			Matrix RC = rotX(thetaC);
			Matrix dRC = drotX(thetaC);
			
			// Get the transform matrices
			Matrix HBase = TBase.times(RBase);
			
			Matrix HA = RA.times(TA);
			Matrix dHA = dRA.times(TA);
			
			Matrix HB = RB.times(TB);
			Matrix dHB = dRB.times(TB);
			
			Matrix HC = RC.times(TC);
			Matrix dHC = dRC.times(TC);

			// Jacobian
			Matrix origin = new Matrix(new double[] { 0, 0, 0, 1 }, 4);
			Matrix dTheta1 = HBase.times(dHA).times(HB).times(HC).times(origin);
			Matrix dTheta2 = HBase.times(HA).times(dHB).times(HC).times(origin);
			Matrix dTheta3 = HBase.times(HA).times(HB).times(dHC).times(origin);

			Matrix J = new Matrix(
			new double[][] {
					{ dTheta1.get(0, 0), dTheta2.get(0, 0), dTheta3.get(0, 0) },
					{ dTheta1.get(1, 0), dTheta2.get(1, 0), dTheta3.get(1, 0) },
					{ dTheta1.get(2, 0), dTheta2.get(2, 0), dTheta3.get(2, 0) }, });
			
			// Compute dX
			Matrix dX;
			try {
				dX = J.solve(dY).times(0.5);
			} catch (RuntimeException e) {
				System.err.println("Warning: " + e.getMessage() + " Solving the gradient descent problem instead");
				dX = J.transpose().times(dY).times(0.5);
			}
			theta = theta.plus(dX);

			thetaA = theta.get(0, 0) % (2 * Math.PI);
			thetaB = theta.get(1, 0) % (2 * Math.PI);
			thetaC = theta.get(2, 0) % (2 * Math.PI);

			theta = new Matrix(new double[] { thetaA, thetaB, thetaC }, 3);
		}
		return toIntArray(theta.times(180 / Math.PI));
	}

	/**
	 * Homogenous coordinate matrix generator for translation
	 * @param p point to translate by
	 * @return the translation matrix that moves to that point.
	 */
	public static Matrix translate(Point3D p) {
		return new Matrix(new double[][] {
				{ 1, 0, 0, p.x},
				{ 0, 1, 0, p.y},
				{ 0, 0, 1, p.z},
				{ 0, 0, 0, 1},
		});
	}
	/**
	 * Homogenous coordinate matrix generator for translation
	 * @param x
	 * @param y
	 * @param z
	 * @return the translation matrix that moves to that point.
	 */
	public static Matrix translate(double x, double y, double z) {
		return new Matrix(new double[][] {
				{ 1, 0, 0, x},
				{ 0, 1, 0, y},
				{ 0, 0, 1, z},
				{ 0, 0, 0, 1},
		});
	}
	
	/**
	 * Homogenous coordinate matrix generator for rotation in X
	 * @param theta amount to rotate by
	 * @return the rotation matrix that rotates that angle
	 */
	public static Matrix rotX(double theta) {
		return new Matrix(new double[][] {
				{ 1, 0, 0, 0 },
				{ 0, Math.cos(theta), -Math.sin(theta), 0 },
				{ 0, Math.sin(theta), Math.cos(theta), 0 },
				{ 0, 0, 0, 1 } });
	}
	/**
	 * Homogenous coordinate matrix generator for rotation in Y
	 * @param theta amount to rotate by
	 * @return the rotation matrix that rotates that angle
	 */
	public static Matrix rotY(double theta) {
		return new Matrix(new double[][] {
				{ Math.cos(theta), 0, -Math.sin(theta), 0 },
				{ 0, 1, 0, 0 },
				{ Math.sin(theta), 0, Math.cos(theta), 0 },
				{ 0, 0, 0, 1 } });
	}
	/**
	 * Homogenous coordinate matrix generator for rotation in Z
	 * @param theta amount to rotate by
	 * @return the rotation matrix that rotates that angle
	 */
	public static Matrix rotZ(double theta) {
		return new Matrix(new double[][] {
				{ Math.cos(theta), -Math.sin(theta), 0, 0},
				{ Math.sin(theta),  Math.cos(theta), 0, 0 },
				{ 0, 0, 1, 0 },
				{ 0, 0, 0, 1 } });
	}
	/**
	 * Homogenous coordinate matrix generator for the derivative
	 * of the rotation matrix in X
	 * @param theta amount to rotate by
	 * @return the derivative of the rotation matrix that rotates that angle
	 */
	public static Matrix drotX(double theta) {
		return new Matrix(new double[][] {
				{ 0, 0, 0, 0 },
				{ 0,-Math.sin(theta), -Math.cos(theta), 0 },
				{ 0, Math.cos(theta), -Math.sin(theta), 0 },
				{ 0, 0, 0, 0 } });
	}
	/**
	 * Homogenous coordinate matrix generator for the derivative
	 * of the rotation matrix in Y
	 * @param theta amount to rotate by
	 * @return the derivative of the rotation matrix that rotates that angle
	 */
	public static Matrix drotY(double theta) {
		return new Matrix(new double[][] {
				{ -Math.sin(theta), 0, -Math.cos(theta), 0 },
				{ 0, 0, 0, 0 },
				{ Math.cos(theta), 0, -Math.sin(theta), 0 },
				{ 0, 0, 0, 0 } });
	}
	/**
	 * Homogenous coordinate matrix generator for the derivative
	 * of the rotation matrix in Z
	 * @param theta amount to rotate by
	 * @return the derivative of the rotation matrix that rotates that angle
	 */
	public static Matrix drotZ(double theta) {
		return new Matrix(new double[][] {
				{ -Math.sin(theta), -Math.cos(theta), 0, 0 },
				{ Math.cos(theta), -Math.sin(theta), 0, 0 },
				{ 0, 0, 0, 0 },
				{ 0, 0, 0, 0 } });
	}
	
	/**
	 * Converts a matrix to a string for convenient debugging and output.
	 * @param M matrix to print
	 * @return string representing the matrix
	 */
	protected static String matrixToString(Matrix M) {
		StringBuilder res = new StringBuilder("[\n");
		for (double[] row : M.getArray()) {
			String sep = "\t";
			for (double col : row) {
				res.append(sep);
				if (col == 0)
					res.append("      0");
				else
					res.append(String.format("%1.5f", col));
				sep = ",\t";
			}
			res.append('\n');
		}
		res.append("]");
		return res.toString();
	}

	/**
	 * Converts a matrix (vector) into an integer array, rounding
	 * to the nearest integer.
	 * @param vector the vector to convert.
	 */
	private static int[] toIntArray(Matrix vector) {
		double[] array = vector.getColumnPackedCopy();
		int[] result = new int[array.length];
		for (int i = 0; i < array.length; i++) {
			result[i] = (int) Math.round(array[i]);
		}
		return result;
	}

	/**
	 * Gives one update step, d(theta), for the given target point and theta0
	 * values
	 * 
	 * @param error
	 * @param angles rotation amount in radians
	 * @return one step update amount for the numerical step.
	 */
	public static Matrix getInverseNumerical2DStep(Matrix error, Matrix angles) {
		// Perturb the angle to avoid singular points
		angles = angles.plus(Matrix.random(angles.getRowDimension(), 1));

		double[][] Jac = new double[2][2];
		// dx/d0
		Jac[0][0] = -link_lengths[0] * Math.sin(angles.get(0, 0))
				- link_lengths[1]
				* Math.sin(angles.get(0, 0) + angles.get(1, 0));
		Jac[0][1] = -link_lengths[1]
				* Math.sin(angles.get(0, 0) + angles.get(1, 0));

		Jac[1][0] = link_lengths[0] * Math.cos(angles.get(0, 0))
				+ link_lengths[1]
				* Math.cos(angles.get(0, 0) + angles.get(1, 0));
		Jac[1][1] = link_lengths[1]
				* Math.cos(angles.get(0, 0) + angles.get(1, 0));

		Matrix J = new Matrix(Jac);

		// Solve for delta Theta
		return J.solve(error);
	}

	/**
	 * Given two points, returns a list of target points for the robot to move
	 * towards along a line, or null if the target has been reached.
	 * 
	 * @param start
	 * @param end
	 * @param resolution
	 *            the number of intermediate points to generate
	 * @return
	 */
	public static Point3D[] createLinePath(Point3D start, Point3D end) {
		double len = start.distance(end);
		int resolution = (int) (len / step_size);

		Point3D[] points = new Point3D[resolution + 1];

		double dx = step_size / len * (end.x - start.x);
		double dy = step_size / len * (end.y - start.y);
		double dz = step_size / len * (end.z - start.z);

		for (int i = 1; i <= resolution; i++) {
			points[i - 1] = new Point3D(start.x + dx * i, start.y + dy * i,
					start.z + dz * i);
		}
		points[resolution] = end;

		return points;
	}

	/**
	 * Given a 2D image, returns a path that traces the image.
	 * 
	 * @param current
	 * @return
	 */
	public static Point3D[] createPicturePath(Point3D current, File image) {
		return null;
	}
}
