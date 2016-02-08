package ca.ualberta;
import java.io.File;

import lejos.utility.Matrix;

public class Kinematics {
	
	public static int step_size = 10;
	public static final int[] link_lengths = {135, 105, 85};	// the length of each joint in mm.
	
	/**
	 * Given a set of theta values for the joint angles, returns
	 * the (x,y)-point location of the end effector
	 * @param theta
	 * @return
	 */
	public static Point3D forwardKinematics(int[] theta) {
		double thetaA = Math.toRadians(theta[0]);
		double thetaB = Math.toRadians(theta[1]);
		double thetaC = 0;
		if (theta.length > 2){
			thetaC = Math.toRadians(theta[1]);
		}

		// == Base Parameters
		Matrix TBase = new Matrix(new double[][]{
			{1,0,0,0},
			{0,1,0,0},
			{0,0,1,link_lengths[2]},
			{0,0,0,1},
		});
		Matrix RBase = new Matrix(new double[][]{
			{1,0,0,0},
			{0,1,0,0},
			{0,0,1,0},
			{0,0,0,1}
		});
		
		// == Motor A Parameters
		Matrix TA = new Matrix(new double[][]{
			{1,0,0,link_lengths[0]},
			{0,1,0,0},
			{0,0,1,0},
			{0,0,0,1},
		});
		Matrix RA = new Matrix(new double[][]{
			{Math.cos(thetaA), -Math.sin(thetaA), 0, 0},
			{Math.sin(thetaA),  Math.cos(thetaA), 0, 0},
			{0,0,1,0},
			{0,0,0,1}
		});
		// == Motor B Parameters
		Matrix TB = new Matrix(new double[][]{
			{1,0,0,link_lengths[1]},
			{0,1,0,0},
			{0,0,1,0},
			{0,0,0,1},
		});
		Matrix RB = new Matrix(new double[][]{
			{Math.cos(thetaB), -Math.sin(thetaB), 0, 0},
			{Math.sin(thetaB),  Math.cos(thetaB), 0, 0},
			{0,0,1,0},
			{0,0,0,1}
		});
		// == Motor C Parameters
		Matrix TC = new Matrix(new double[][]{
			{1,0,0,0},
			{0,1,0,0},
			{0,0,1,-link_lengths[2]},
			{0,0,0,1},
		});
		Matrix RC = new Matrix(new double[][]{
			{Math.cos(thetaC), 0, -Math.sin(thetaC), 0},
			{0,1,0,0},
			{Math.sin(thetaC), 0,  Math.cos(thetaC), 0},
			{0,0,0,1}
		});
		
		Matrix end = RBase.times(TBase)
				.times(RA).times(TA)
				.times(RB).times(TB)
				.times(RC).times(TC)
				.times(new Matrix(new double[]{0,0,0,1},4));
		
		return new Point3D(end);
	}
	
	/**
	 * Given 3 points of two intersecting lines find the angle they intersect at
	 * @param c intersecting point
	 * @param a point from first line
	 * @param b poit from other line
	 * @return angle between the two lines at point c
	 */
	public static int getAngleBWlines(Point3D c, Point3D a, Point3D b){
		// get absolute side lengths
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
	 * finds center point of the circle given only 3 points on that circle(so any arc)
	 * @param a leftmost point
	 * @param b a point on the arc between a and b
	 * @param c rightmost point
	 * @return center of circle
	 */
	// see math reference: http://www.regentsprep.org/regents/math/geometry/gcg6/RCir.htm
	public static Point3D getRadiusArc(Point3D a, Point3D b, Point3D c){
		double slope_ab = (b.x - a.x / b.y - a.y);
		double slope_bc = (c.x - b.x / c.y - b.y);
		// line_y1= slope_ab*(x-a.x) + a.y
		// line_y2= slope_bc*(x-b.x) + b.y
		// intersection of these lines is at circle center
		double midx1 = (b.x + a.x)/2;
		double midy1 = (b.y + a.y)/2;
		double midx2 = (c.x + b.x)/2;
		double midy2 = (c.y + b.y)/2;
		double mid1_slope = -1/slope_ab;
		double mid2_slope = -1/slope_bc;
		//Point3D midpoint1 = new Point3D(midx1, midy1);
		//Point3D midpoint2 = new Point3D(midx2, midy2);
		
		//CHECK THIS FOR FUCK UPS!!!
		//mid1_slope*(x-midx1) + midy1 = mid2_slope*(x-midx2) + midy2 solve for x
		double centerx = ( (mid1_slope*midx1 - mid2_slope*midx2 - midy1 + midy2) 
							/ (mid1_slope - mid2_slope) );
		double centery = mid1_slope*(centerx-midx1) + midy1;
		Point3D center = new Point3D(centerx, centery);
		
		return center;
	}

//	public static int[] inverseKinematics(Point3D target) {
//		return inverseAnalyticKinematics(target);
//	}

	public static int[] inverseKinematics(Point3D target, int[] theta0) {
		return inverseNumericalKinematics(target, theta0);
	}
	
	/**
	 * Given a point, computes and returns the joint angles
	 * required to achieve the target
	 * @param target
	 * @return final theta values
	 */
	public static int[] inverseAnalyticKinematics2D(Point3D target) {
		double[] angles = new double[3];
		double r = target.x*target.x + target.y*target.y;
		double l1 = link_lengths[0];
		double l2 = link_lengths[1];
		// if absolute value of r-l1*l1-l2*l2 is greater than one---> NAN
		// so only valid for points in: 30 < sqrt(x^2+y^2) < 240
		angles[1] = Math.acos((r-l1*l1-l2*l2) / (2*l1*l2));
		angles[0] = Math.asin(-l2*Math.sin(angles[1])/(Math.sqrt(r)))
										+ Math.atan2(target.y, target.x);
		
		int[] theta = new int[angles.length];
		for (int i = 0; i < theta.length; i++) {
			theta[i] = (int) Math.round(Math.toDegrees(angles[i]));
		}
		//System.out.format("theta[1]= %d \ntheta[0]= %d\n", theta[1], theta[0]);
		// Invalid for 3D inverse kinematics, so zero out the last entry
		angles[2] = 0;
		return theta;
	}
	
	/**
	 * Solves the inverse kinematics problem using
	 * Newton's method.
	 * @param target
	 * @param theta0
	 * @return
	 */
	public static int[] inverseNumericalKinematics2D(Point3D target, int[] theta0) {
		double[] theta = new double[theta0.length];
		int[] round = new int[theta0.length];
		
		// Copy theta0 to angles
		for (int j = 0; j < theta.length; j++)
			theta[j] = (double)Math.toRadians(theta0[j]);

		Matrix angles = new Matrix(theta, theta.length);
		
		for (int i = 0; i < 100; i++) {
			// Get the error term
			round = toIntArray(angles.times(180d/Math.PI));
			Point3D actual = forwardKinematics(round);
			Matrix error = new Matrix(
					new double[]{target.x - actual.x, target.y - actual.y, 0}, 
					3);

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
		return toIntArray(angles.times(180d/Math.PI));
	}
	
	public static int[] inverseNumericalKinematics(Point3D target, int[] theta0) {
		double thetaA = Math.toRadians(theta0[0]);
		double thetaB = Math.toRadians(theta0[1]);
		double thetaC = Math.toRadians(theta0[2]);
		
		Matrix theta = new Matrix(new double[] {thetaA, thetaB, thetaC}, 3);
		
		for (int i = 0; i < 100; i++) {
			// Error term
			Point3D actual = forwardKinematics(toIntArray(theta.times(180/Math.PI)));
			Matrix dY = target.toMatrix().minus(actual.toMatrix());
			
			if (dY.normF() < (TestKinematics.dist_thresh))
				break;
			
			// == Motor A Parameters
			Matrix TA = new Matrix(new double[][]{
				{1,0,0,link_lengths[0]},
				{0,1,0,0},
				{0,0,1,0},
				{0,0,0,1},
			});
			Matrix RA = new Matrix(new double[][]{
				{Math.cos(thetaA), -Math.sin(thetaA), 0, 0},
				{Math.sin(thetaA),  Math.cos(thetaA), 0, 0},
				{0,0,1,0},
				{0,0,0,1}
			});
			Matrix dRA = new Matrix(new double[][]{
				{-Math.sin(thetaA), -Math.cos(thetaA), 0, 0},
				{ Math.cos(thetaA), -Math.sin(thetaA), 0, 0},
				{0,0,1,0},
				{0,0,0,1}
			});
			
			Matrix HA = TA.times(RA);
			Matrix dHA = TA.times(dRA);
			
			// == Motor B Parameters
			Matrix TB = new Matrix(new double[][]{
				{1,0,0,link_lengths[1]},
				{0,1,0,0},
				{0,0,1,0},
				{0,0,0,1},
			});
			Matrix RB = new Matrix(new double[][]{
				{Math.cos(thetaB), -Math.sin(thetaB), 0, 0},
				{Math.sin(thetaB),  Math.cos(thetaB), 0, 0},
				{0,0,1,0},
				{0,0,0,1}
			});
			Matrix dRB = new Matrix(new double[][]{
				{-Math.sin(thetaB), -Math.cos(thetaB), 0, 0},
				{ Math.cos(thetaB), -Math.sin(thetaB), 0, 0},
				{0,0,1,0},
				{0,0,0,1}
			});
			
			Matrix HB = TB.times(RB);
			Matrix dHB = TB.times(dRB);
			
			// == Motor C Parameters
			Matrix TC = new Matrix(new double[][]{
				{1,0,0,0},
				{0,1,0,0},
				{0,0,1,-link_lengths[2]},
				{0,0,0,1},
			});
			Matrix RC = new Matrix(new double[][]{
				{Math.cos(thetaC), 0, -Math.sin(thetaC), 0},
				{0,1,0,0},
				{Math.sin(thetaC), 0,  Math.cos(thetaC), 0},
				{0,0,0,1}
			});
			Matrix dRC = new Matrix(new double[][]{
				{-Math.sin(thetaC), 0, -Math.cos(thetaC), 0},
				{0,1,0,0},
				{Math.cos(thetaC), 0,  -Math.sin(thetaC), 0},
				{0,0,0,1}
			});
			
			Matrix HC = TB.times(RC);
			Matrix dHC = TC.times(dRC);
			
			// Jacobian
			Matrix origin = new Matrix(new double[]{0,0,0,1}, 4);
			Matrix dTheta1 = dHA.times(HB).times(HC).times(origin);
			Matrix dTheta2 = HA.times(dHB).times(HC).times(origin);
			Matrix dTheta3 = HA.times(HB).times(dHC).times(origin);
			
			Matrix J = new Matrix(new double[][] {
				{dTheta1.get(0, 0), dTheta2.get(0, 0), dTheta3.get(0,0)},
				{dTheta1.get(1, 0), dTheta2.get(1, 0), dTheta3.get(1,0)},
				{dTheta1.get(2, 0), dTheta2.get(2, 0), dTheta3.get(2,0)},
			});
			
			// Solve the inverse problem
			Matrix dX = J.solve(dY).times(0.5);
			
			theta = theta.plus(dX);

			thetaA = theta.get(0, 0) % (2*Math.PI);
			thetaB = theta.get(1, 0) % (2*Math.PI);
			thetaC = theta.get(2, 0) % (2*Math.PI);
			
			theta = new Matrix(new double[] {thetaA, thetaB, thetaC},3);
		}
		return toIntArray(theta.times(180/Math.PI));
	}
	
//	public static int[] inverseNumericalKinematics(Point3D target, int[] theta0) {
//		// Convert theta0 to matrix
//		double theta[] = new double[theta0.length];
//		for (int i=0; i<theta.length; i++)
//			theta[i] = Math.toRadians(theta0[i]);
//		
//		// Initial angles
//		Matrix angles = new Matrix(theta, theta.length);
//		
//		// Get f0, deltaF
//		Point3D ppos = target;
//		Point3D actual = forwardKinematics(toIntArray(angles.times(180d/Math.PI)));
//		System.out.println("epos = " + actual);
//		
//		Matrix deltaTarget = actual.minus(target).toMatrix();
//		System.out.println("epos-target = " + matrixToString(deltaTarget));
//		
//		// Stop if we're already in range.
//		if (deltaTarget.normF() < Math.sqrt(TestKinematics.dist_thresh))
//			return theta0;
//		
//		// Secant approximation
//		Point3D dt1 = forwardKinematics(new int[]{theta0[0]+1, theta0[1], theta0[2]})
//				.minus(forwardKinematics(new int[]{theta0[0], theta0[1], theta0[2]}));
//		
//		Point3D dt2 = forwardKinematics(new int[]{theta0[0], theta0[1]+1, theta0[2]})
//				.minus(forwardKinematics(new int[]{theta0[0], theta0[1], theta0[2]}));
//		
// 		Point3D dt3 = forwardKinematics(new int[]{theta0[0], theta0[1], theta0[2]+1})
//				.minus(forwardKinematics(new int[]{theta0[0], theta0[1], theta0[2]}));
//
//		// Get an initial estimate for the Jacobian
//		Matrix B = new Matrix(new double[][] {
//			{dt1.x, dt2.x, dt3.x},
//			{dt1.y, dt2.y, dt3.y},
//			{dt1.z, dt2.z, dt3.z},
//		}).times(Math.PI/180d); // divide out that d(radian) term
//		System.out.println("Bk = " + matrixToString(B));
//		
//		// Loop until good enough
//		for (int i = 0; i < 100; i++) {
//			// Add some noise to avoid singularities
////			B = B.plus(B.arrayTimes((Matrix.random(3, 3).minus(Matrix.random(3, 3)))
////						.times(10e-13)));
//			// Solve for the angle update term
//			LUDecomposition LU = new LUDecomposition(B);
//			if (!LU.isNonsingular()) {
//				System.out.println("Matrix is singular!");
//				break;
//			}
//			Matrix deltaX = B.solve(deltaTarget.times(-1));
//			System.out.println("sk = " + matrixToString(deltaX));
//			
//			// Update the angles
//			angles = angles.plus(deltaX);
//			
//			// Modulo 2pi
//			for (int e = 0; e < angles.getColumnPackedCopy().length; e++) {
//				angles.set(e, 0, (angles.get(e, 0)+(2*Math.PI)) % (2*Math.PI));
//			}
//			System.out.println("theta = " + matrixToString(angles));
//			
//			// Update position
//			ppos = actual;
//			actual = forwardKinematics(toIntArray(angles.times(180d/Math.PI)));
//			System.out.println("epos = " + actual);
//
//			// Update the error term
//			deltaTarget = actual.minus(target).toMatrix();
//			// Check if within threshold
//			if (deltaTarget.normF() < Math.sqrt(TestKinematics.dist_thresh)) {
//				break;
//			}
//			
//			// Update the change in Y = (actual - target) - (previous - target)
//			Matrix deltaY = actual.minus(ppos).toMatrix();
//			System.out.println("yk = " + matrixToString(deltaY));
//			
//			// Update the Jacobian
//			B = updateBroydenStep(B, deltaY, deltaX);
//			System.out.println("Bk = " + matrixToString(B));
//			
//		}
//		
//		return toIntArray(angles.times(180d/Math.PI));
//	}
	
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
	 * Updates the Broyden approximation for the Jacobian.
	 * Note: if you swap deltaF and deltaX and have B
	 * be the approximate inverse Jacobian, then this
	 * update will return an approximation for the inverse
	 * Jacobian, per the "bad" Broyden method.
	 * @return
	 */
	public static Matrix updateBroydenStep(Matrix B, Matrix deltaF, Matrix deltaX) {
		// (df - B*dx) / (dx*B*dF) 
		Matrix num = deltaF.minus(B.times(deltaX));
		double denom = 1 / Math.pow(deltaX.normF(),2);
		
		Matrix outer = num.times(deltaX.transpose());
		
		Matrix dB = outer.times(denom);
		
		return B.plus(dB);
	}
	
	private static int[] toIntArray(Matrix vector) {
		double[] array = vector.getColumnPackedCopy();
		int[] result = new int[array.length];
		for (int i = 0; i < array.length; i++) {
			result[i] = (int) Math.round(array[i]);
		}
		return result;
	}

	/**
	 * Gives one update step, d(theta), for the given target point and
	 * theta0 values
	 * @param error
	 * @param angles, in radians
	 * @return
	 */
	public static Matrix getInverseNumerical2DStep(Matrix error, Matrix angles) {
		// Perturb the angle to avoid singular points
		angles = angles.plus(Matrix.random(angles.getRowDimension(), 1));

		
		double[][] Jac = new double[3][2];
		// dx/d0
		Jac[0][0] = -link_lengths[0]*Math.sin(angles.get(0,0))
				-link_lengths[1]*Math.sin(angles.get(0,0) + angles.get(1,0));
		Jac[0][1] = -link_lengths[1]*Math.sin(angles.get(0,0) + angles.get(1,0));
		
		Jac[1][0] = link_lengths[0]*Math.cos(angles.get(0,0))
				+link_lengths[1]*Math.cos(angles.get(0,0) + angles.get(1,0));
		Jac[1][1] = link_lengths[1]*Math.cos(angles.get(0,0) + angles.get(1,0));
		
		Jac[2][0] = 0;
		Jac[2][1] = 0;
		
		Matrix J = new Matrix(Jac);
		
		// Solve for delta Theta
		return J.solve(error);
	}
	
	/**
	 * Gives one update step, d(theta), for the given target point and
	 * theta0 values
	 * @param error
	 * @param angles, in radians
	 * @return
	 */
	public static double[] getInverseNumericalStep(double[] error, double[] angles) {
		return getInverseNumerical2DStep(new Matrix(error,error.length), new Matrix(angles,angles.length)).getColumnPackedCopy();
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
		return getInverseNumericalStep(error, thetas);
	}
	
	/**
	 * Given two points, returns a list of target points for the robot
	 * to move towards along a line, or null if the target has been reached.
	 * @param start
	 * @param end
	 * @param resolution the number of intermediate points to generate 
	 * @return
	 */
	public static Point3D[] createLinePath(Point3D start, Point3D end) {
		double len = start.distance(end);
		int resolution = (int) (len / step_size);
		
		Point3D[] points = new Point3D[resolution+1];
		
		double dx = step_size/len*(end.x-start.x);
		double dy = step_size/len*(end.y-start.y);
		double dz = step_size/len*(end.z-start.z);
		
		for (int i = 1; i <= resolution; i++) {
			points[i-1] = new Point3D(start.x+dx*i, start.y+dy*i, start.z+dz*i);
		}
		points[resolution] = end;
		
		return points;
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
