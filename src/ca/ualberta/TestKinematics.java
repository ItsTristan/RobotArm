package ca.ualberta;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import lejos.utility.Matrix;

public class TestKinematics extends Kinematics {

	public static final int dist_thresh = 2;	// Maximum number of mm to be off by in inverse kinematics

	final int L1 = link_lengths[0];
	final int L2 = link_lengths[1];
	final int L3 = link_lengths[2];
	
	final double root2 = Math.sqrt(2);
	final double root3 = Math.sqrt(3);
	
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
		Assert.assertEquals(new Point3D(L1+L2,0), actual);

		// Base at 90
		actual = forwardKinematics(new int[]{90, 0, 0});
		Assert.assertEquals(new Point3D(0,L1+L2), actual);
		
		// Both at 45/45 (positive x, positive y)
		actual = forwardKinematics(new int[]{45, 45, 0});
		Point3D target = new Point3D(L1/root2, L2+L1/root2);
		Assert.assertTrue("expected: "+target+", got: "+actual,
				target.distance(actual) <= 2);
		
		// Negative values (positive x, negative y)
		actual = forwardKinematics(new int[]{-45, -45, 0});
		target = new Point3D(L1/root2, -(L2+L1/root2));
		Assert.assertTrue("expected: "+target+", got: "+actual,
				target.distance(actual) <= 2);
		
		// Far angle case (negative x, positive y)
		actual = forwardKinematics(new int[]{180-45, -45, 0});
		target = new Point3D(-L1/root2, L2+L1/root2);
		Assert.assertTrue("expected: "+target+", got: "+actual,
				target.distance(actual) <= 2);

		// 3 Dimensions
		actual = forwardKinematics(new int[]{0,0,180});
		target = new Point3D(L1+L2,0,2*L3);
		Assert.assertTrue("expected: "+target+", got: "+actual,
				target.distance(actual) <= 2);

		// 3 Dimensions in all dimensions
		actual = forwardKinematics(new int[]{90,90,90});
		target = new Point3D(-L2,L1-L3,L3);
		Assert.assertTrue("expected: "+target+", got: "+actual,
				target.distance(actual) <= 2);
		
		// Not as trivial 3D case
		actual = forwardKinematics(new int[]{45,45,45});
		target = new Point3D(L1/root2-L3/root2, L2+L1/root2, L3-L3/root2);
		Assert.assertTrue("expected: "+target+", got: "+actual,
				target.distance(actual) <= 2);
		
	}
	
	@Test
	public void testInverseAnalyticKinematics() {
		// Stupid case
		assertInverseAnalytical(new Point3D(240,0), new int[]{0,0,0});
		
		// Slight variation
		assertInverseAnalytical(new Point3D(240,0), new int[]{0,0,0});
		
		// Slight variation
		assertInverseAnalytical(new Point3D(231,59), new int[]{10,10,0});
		
		// Base at 90
		assertInverseAnalytical(new Point3D(0,240), new int[]{90,0,0});
		
		// Both at 45/45 (positive x, positive y)
		assertInverseAnalytical(new Point3D(L1/root2, L2+L1/root2),
					new int[]{45,45,0});
		
		// Negative values (positive x, negative y)
		assertInverseAnalytical(new Point3D(L1/root2, -(L2+L1/root2)),
					new int[]{-45, -45,0});
		
		// Far angle case (negative x, positive y)
		assertInverseAnalytical(new Point3D(-L1/root2, L2+L1/root2),
				new int[]{180-45, -45,0});
		
		// Degenerate test case
		assertInverseAnalytical(new Point3D(30,30),
				new int[]{98,180,0});
		
		// Arbitrary example
		assertInverseAnalytical(new Point3D(100,100),
				new int[]{9,109,0});
	}
	
	@Test
	public void testInverseNumericalKinematics() {
		// Loop to check for consistency
		for (int i = 0; i < 5; i++)
		{
			// Stupid case
			assertInverseNumerical(new Point3D(240,0), new int[]{0,0,0}, new int[]{0,0,0});
			
			// Slight variation
			assertInverseNumerical(new Point3D(240,0), new int[]{0,0,0}, new int[]{2,2,0});
			
			// Slight variation
			assertInverseNumerical(new Point3D(231,59), new int[]{10,10,0}, new int[]{2,2,0});
		
			// Base at 90
			assertInverseNumerical(new Point3D(0,240), new int[]{90,0,0}, new int[]{82,10,0});
			
			// Both at 45/45 (positive x, positive y)
			assertInverseNumerical(new Point3D(L1/root2, L2+L1/root2),
					new int[]{45,45,0}, new int[]{42,42,0});
			
			// Negative values (positive x, negative y)
			assertInverseNumerical(new Point3D(L1/root2, -(L2+L1/root2)),
						new int[]{-45, -45,0}, new int[]{-20,-60,0});
			
			// Far angle case (negative x, positive y)
			assertInverseNumerical(new Point3D(-L1/root2, L2+L1/root2),
					new int[]{180-45, -45,0}, new int[]{180, -30,0});
			
			// Degenerate test case
			assertInverseNumerical(new Point3D(30,30),
					new int[]{98,180,0}, new int[]{90, 170,0});
			
			// Arbitrary example
			assertInverseNumerical(new Point3D(100,100),
					new int[]{9,109,0}, new int[]{0, 80,0});
			
			// 3 Dimensions
			assertInverseNumerical(new Point3D(L1+L2,0,2*L3),
					new int[]{0,0,180}, new int[]{5,-5,187});
			
			// 3 Dimensions in all dimensions
			assertInverseNumerical(new Point3D(-L2,L1-L3,L3),
					new int[]{90,90,90}, new int[]{97,85,100});
			
			// Not as trivial 3D case
			assertInverseNumerical(new Point3D(L1/root2-L3/root2, L2+L1/root2, L3-L3/root2),
					new int[]{45,45,45}, new int[]{37,48,52});
		}
	}

	/**
	 * Asserts that the analytic solution is close enough
	 * @param target
	 * @param expected
	 */
	private void assertInverseAnalytical(Point3D target, int[] expected) {
		int[] actual = inverseAnalyticKinematics2D(target);
		checkInverse(target, expected, actual);
	}
	/**
	 * Asserts that the numerical solution is close enough
	 * @param target
	 * @param expected
	 * @param hint
	 */
	private void assertInverseNumerical(Point3D target, int[] expected, int[] hint) {
		int[] actual = inverseNumericalKinematics(target,hint);
		checkInverse(target, expected, actual);
	}

	/**
	 * Checks that the inverse solution is correct
	 * @param target
	 * @param expected
	 * @param actual
	 */
	private void checkInverse(Point3D target, int[] expected, int[] actual) {
		Point3D where = forwardKinematics(actual);
		Assert.assertTrue("Solution is not close enough. "+
		"Got ("+print(actual)+") → "+where+
		", Expected ("+print(expected)+") → "+target,
				where.distance(target) <= dist_thresh);
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
}
