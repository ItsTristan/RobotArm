package ca.ualberta;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

public class TestKinematics extends Kinematics {

	public static final int dist_thresh = 3;	// Maximum number of mm to be off by in inverse kinematics

	final int L1 = link_lengths[0], L2 = link_lengths[1];
	
	final int scale = 10000;
	final int root2 = (int) (scale*Math.sqrt(2));

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
		actual = forwardKinematics(new int[]{0, 0});
		Assert.assertEquals(new Point3D(240,0), actual);

		// Base at 90
		actual = forwardKinematics(new int[]{90, 0});
		Assert.assertEquals(new Point3D(0,240), actual);
		
		// Both at 45/45 (positive x, positive y)
		actual = forwardKinematics(new int[]{45, 45});
		Point3D target = new Point3D(L1*scale/root2, L2+L1*scale/root2);
		Assert.assertTrue(target+"->"+actual,
				target.distance(actual) <= 2);
		
		// Negative values (positive x, negative y)
		actual = forwardKinematics(new int[]{-45, -45});
		target = new Point3D(L1*scale/root2, -(L2+L1*scale/root2));
		Assert.assertTrue(target+"->"+actual,
				target.distance(actual) <= 2);
		
		// Far angle case (negative x, positive y)
		actual = forwardKinematics(new int[]{180-45, -45});
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
		testInverseKinematics();
	}
	
	public void testInverseKinematics() {
		
		// All tests are inverse of forward case.
		// A test passes if the forward kinematics of 
		// the returned solution is within some threshold
		
		// Stupid case
		assertInverse(new Point3D(240,0), new int[]{0,0});
		
		// Slight variation
		assertInverse(new Point3D(240,0), new int[]{0,0}, new int[]{10,10});

		// Base at 90
		assertInverse(new Point3D(0,240), new int[]{90,0}, new int[]{82,10});
		
		// Both at 45/45 (positive x, positive y)
		assertInverse(new Point3D(L1*scale/root2, L2+L1*scale/root2),
					new int[]{45,45}, new int[]{30,30});
		
		// Negative values (positive x, negative y)
		assertInverse(new Point3D(L1*scale/root2, -(L2+L1*scale/root2)),
					new int[]{-45, -45}, new int[]{-20,-60});
		
		// Far angle case (negative x, positive y)
		assertInverse(new Point3D(-L1*scale/root2, L2+L1*scale/root2),
				new int[]{180-45, -45}, new int[]{180, -30});
		
		assertInverse(new Point3D(10,10),
				new int[]{98, 180}, new int[]{90, 170});
	}
	
	private void assertInverse(Point3D target, int[] expected) {
		assertInverse(target, expected, new int[]{0,0});
	}
	
	private void assertInverse(Point3D target, int[] expected, int[] hint) {
		int[] actual;
		if (using_analytic) {
			actual = inverseAnalyticKinematics(target);
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
}
