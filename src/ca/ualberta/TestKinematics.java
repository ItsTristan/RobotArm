package ca.ualberta;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import lejos.utility.Matrix;

public class TestKinematics extends Kinematics {

	public static final int dist_thresh = 9;	// Maximum number of mm to be off by in inverse kinematics

	final int L1 = link_lengths[0], L2 = link_lengths[1];
	
	final int scale = 10000;
	final int root2 = (int) (scale*Math.sqrt(2));
	
	final boolean verbose = false;

	@Before
	public void setUp() throws Exception {
	}

	@Test
	public void testForwardKinematics() {
	
		// All points verified with basic geometry
		// Assumes default position is [240, 0]
		// and 
		
		Point3D actual;
		
		// Initial Condition
		actual = forwardKinematics(new int[]{0, 0, 0});
		Assert.assertEquals(new Point3D(240,0), actual);

		// Base at 90
		actual = forwardKinematics(new int[]{90, 0, 0});
		Assert.assertEquals(new Point3D(0,240), actual);
		
		// Both at 45/45 (positive x, positive y)
		actual = forwardKinematics(new int[]{45, 45, 0});
		Point3D target = new Point3D(L1*scale/root2, L2+L1*scale/root2);
		Assert.assertTrue(target+"->"+actual,
				target.distance(actual) <= 2);
		
		// Negative values (positive x, negative y)
		actual = forwardKinematics(new int[]{-45, -45, 0});
		target = new Point3D(L1*scale/root2, -(L2+L1*scale/root2));
		Assert.assertTrue(target+"->"+actual,
				target.distance(actual) <= 2);
		
		// Far angle case (negative x, positive y)
		actual = forwardKinematics(new int[]{180-45, -45, 0});
		target = new Point3D(-L1*scale/root2, L2+L1*scale/root2);
		Assert.assertTrue(target+"->"+actual,
				target.distance(actual) <= 2);
		
	}
	
	private boolean using_analytic = false;
	
	@Test
	public void testInverseAnalyticKinematics() {
		using_analytic = true;
		testInverseKinematics();
	}
	
	@Test
	public void testInverseNumericalKinematics() {
		using_analytic = false;
		// Loop to check for consistency
		for (int i = 0; i < 100; i++) {
			testInverseKinematics();
		}
	}
	
	public void testInverseKinematics() {
		
		// All tests are inverse of forward case.
		// A test passes if the forward kinematics of 
		// the returned solution is within some threshold
		
		// Stupid case
		assertInverse(new Point3D(240,0), new int[]{0,0,0}, new int[]{0,0,0});

		// Slight variation
		assertInverse(new Point3D(240,0), new int[]{0,0,0}, new int[]{10,10,0});

		// Slight variation
		assertInverse(new Point3D(231,59), new int[]{10,10,0}, new int[]{2,2,0});
		
		// Base at 90
		assertInverse(new Point3D(0,240), new int[]{90,0,0}, new int[]{82,10,0});
		
		// Both at 45/45 (positive x, positive y)
		assertInverse(new Point3D(L1*scale/root2, L2+L1*scale/root2),
					new int[]{45,45,0}, new int[]{30,30,0});
		
		// Negative values (positive x, negative y)
		assertInverse(new Point3D(L1*scale/root2, -(L2+L1*scale/root2)),
					new int[]{-45, -45,0}, new int[]{-20,-60,0});
		
		// Far angle case (negative x, positive y)
		assertInverse(new Point3D(-L1*scale/root2, L2+L1*scale/root2),
				new int[]{180-45, -45,0}, new int[]{180, -30,0});

		// Degenerate test case
		assertInverse(new Point3D(30,30),
				new int[]{98,180,0}, new int[]{90, 170,0});

		// Arbitrary example
		assertInverse(new Point3D(100,100),
				new int[]{9,109,0}, new int[]{0, 80,0});
	}
	
	private void assertInverse(Point3D target, int[] expected, int[] hint) {
		int[] actual;
		if (using_analytic) {
			actual = inverseAnalyticKinematics2D(target);
		} else {
			actual = inverseNumericalKinematics(target,hint);
		}

		// Check location
		Point3D where = forwardKinematics(actual);
		Assert.assertTrue("Solution is not close enough. "+
		"Got ("+print(actual)+") → "+where+
		", Expected ("+print(expected)+") → "+target,
				where.distance(target) <= dist_thresh);
	}

	private String print(int[] array) {
		StringBuilder result = new StringBuilder();
		String sep = "";
		
		for (int i = 0; i < array.length; i++) {
			result.append(sep);
			result.append(array[i]);
			sep = ", ";
		}
		return result.toString();
	}
	
	@Test
	public void testStepping() {
		int[] theta = new int[]{0,0,0};
		Point3D start = forwardKinematics(theta);
		Point3D final_location = new Point3D(100,100,0);
		
		Point3D[] line = createLinePath(start, final_location);

		if (verbose) System.out.println(start);
		for (Point3D target : line) {
			if (verbose) System.out.println("Target: " + target);
			Assert.assertTrue("Distance to next point is too far:" + start.distance(target), start.distance(target) <= 2*step_size);

			theta = inverseKinematics(target,theta);
			start = forwardKinematics(theta);
			
			if (verbose) System.out.println("New Location: " + start);
		}
		
		theta = inverseKinematics(final_location, theta);
		
		Assert.assertTrue(forwardKinematics(theta).distance(final_location) <= dist_thresh);
	}
	
	@Test
	public void testBroydenUpdate() {
		// No change
		Matrix B0 = new Matrix(new double[][] {
			{1,0,0},
			{0,1,0},
			{0,0,1}
		});
		Matrix deltaF = new Matrix(new double[]{1,1,1},3);
		Matrix deltaX = new Matrix(new double[]{1,1,1},3);
		Matrix B1 = updateBroydenStep(B0, deltaF, deltaX);

		System.out.println(matrixToString(B1));
		System.out.println(matrixToString(B0));
		Assert.assertEquals(B0.normF(), B1.normF(), 10e-14);
		
		// DeltaF = 0
		B0 = new Matrix(new double[][] {
			{1,0,0},
			{0,1,0},
			{0,0,1}
		});
		deltaF = new Matrix(new double[]{0,0,0},3);
		deltaX = new Matrix(new double[]{1,1,1},3);
		B1 = updateBroydenStep(B0, deltaF, deltaX);
		
		Matrix BT = new Matrix(new double[][] {
			{ 2d/3d,  -1d/3d,  -1d/3d},
			{-1d/3d,   2d/3d,  -1d/3d},
			{-1d/3d,  -1d/3d,   2d/3d},
		});
		
		Assert.assertEquals(B1.normF(), BT.normF(), 10e-14);
		
		// Weirder case
		B0 = new Matrix(new double[][] {
			{0,1,2},
			{1,2,0},
			{2,0,1}
		});
		deltaF = new Matrix(new double[]{-1,-2,-3},3);
		deltaX = new Matrix(new double[]{3,2,1},3);
		B1 = updateBroydenStep(B0, deltaF, deltaX);
		
		BT = new Matrix(new double[][] {
			{-1.0714, 0.2857, 1.6429},
			{ 0.0714,-1.2857, 0.3571},
			{-1.1429, 0.5714,-0.7143},
		});

		Assert.assertEquals(B1.normF(), BT.normF(), 10e-4);
		
	}
}
