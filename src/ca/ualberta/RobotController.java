package ca.ualberta;


import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;

public class RobotController {
	
	private static RegulatedMotor m_motorA;
	private static RegulatedMotor m_motorB;
	private static RegulatedMotor m_motorC;

	private static final int defaultDelay = 250;
	private static final int motor_speed = 60;
	private static final int motor_accel = 20;
	
	private static final double[] gear_ratios = {15d/30d, 1d};

	private static EV3TouchSensor touchSensor;
	
	public static void initializeMotorZero() {
		getMotorA().resetTachoCount();
		getMotorB().resetTachoCount();
	}
	
	/***
	 * Relaxes motors to be manipulated by hand again
	 */
	public static void relaxMotors() {
		RegulatedMotor motorA = getMotorA();
		RegulatedMotor motorB = getMotorB();
		motorA.flt();
		motorB.flt();
	}
	/***
	 * Moves the end effector to the point (x,y)
	 * @param x
	 * @param y
	 */
	public static void moveTo(double x, double y) {
		moveTo(new Point3D(x,y));
	}
	
	/**
	 * Moves the end effector to the point p
	 * @param p
	 */
	public static void moveTo(Point3D p) {
		//System.out.println("Move to " + p);
		moveAnalytic(p);
	}
	
	/**
	 * Moves to start location then draws line from there to the final location
	 * @param start_location
	 * @param final_location
	 */
	public static void drawLineBW(Point3D start, Point3D end) {
		System.out.format("start: %f,%f \nend: %f,%f \n", start.x, start.y, end.x, end.y);
		Point3D[] line = Kinematics.createLinePath(start, end);	
		for (Point3D target : line) {
			moveTo(target);
		}
	}
	/**
	 * Given 2 end points of an arc and any other point on the arc can step 
	 * along and draw the entire arc
	 * @param start 
	 * @param mid is any point between start and end that lies on the arc
	 * @param end
	 */
	public static void drawArcLine(Point3D start, Point3D mid, Point3D end){
		//WAIT HOW DO WE KNOW IF MOVING LEFT TO RIGHT IS INCREASING/DECREASING IN COORDINATES?
		Point3D center = Kinematics.getRadiusArc(start, mid, end);
		
		double radius = center.distance(start);
		System.out.println("radius: " + radius);
		System.out.println("center: " + center);
		double arc_angle = Math.toRadians(Kinematics.getAngleBWlines(center, start, end));
		double step_size = arc_angle/20; //do: pi, 19/20pi, 18/20pi,...., 1/20pi, 0pi
		moveTo(start);
		
		Point3D circle_start = new Point3D(-1,0);
		for (int i = 20; i > 0; i--){
			double next_x = radius*Math.cos(step_size*i);
			double next_y = radius*Math.sin(step_size*i);
			double deltax = next_x - circle_start.x;
			double deltay = next_y - circle_start.y;
			moveTo(start.x + deltax, start.y + deltay);
		}
	}
	
	/**
	 * Moves the end effector to the point p
	 * using an analytic solution
	 * @param p
	 */
	public static void moveAnalytic(Point3D p) {
		int[] theta = Kinematics.inverseAnalyticKinematics2D(p);
		rotateTo(theta);
	}
	/**
	 * Moves the end effector to the point p
	 * using an analytic solution
	 * @param p
	 */
	public static void moveNumerical(Point3D p) {
		System.out.println("Numeric Sol.");
		System.out.println("Point:" + p.x + "," + p.y);
		int[] theta = Kinematics.inverseKinematics(p, getJointAngles());
		rotateTo(theta);
	}
	
	public static void rotateTo(int[] theta) {
		rotateTo(theta, defaultDelay);
	}
	
	/**
	 * Rotates the joints to the given angles, returning
	 * the point of the end effector
	 * @param theta
	 * @param delay
	 * @return
	 */
	public static void rotateTo(int[] theta, int delay) {
		RegulatedMotor motorA = getMotorA();
		RegulatedMotor motorB = getMotorB();	

		motorA.setAcceleration(motor_accel);
		motorB.setAcceleration(motor_accel);

		motorA.setSpeed(motor_speed);
		motorB.setSpeed(motor_speed);
		
		motorA.rotateTo((int) ( theta[0] / gear_ratios[0]),true);
		motorB.rotateTo((int) ( theta[1] / gear_ratios[1]),false);	// last one should be false
		
		Delay.msDelay(delay);
	}
	
	/**
	 * Returns the current location of the end effector,
	 * accounting for the gear ratios.
	 * @return
	 */
	public static Point3D getLocation() {
		return Kinematics.forwardKinematics(getJointAngles());
	}

	/**
	 * Returns the current joint angles, accounting for the gear ratios.
	 * @return
	 */
	public static int[] getJointAngles() {
		return new int[] {
				(int) (getMotorA().getTachoCount()*gear_ratios[0]),
				(int) (getMotorB().getTachoCount()*gear_ratios[1])
		};
	}
	
	/**
	 * Stalls execution until the touch sensor is pressed then released
	 */
	public static void waitForTouch() {
		waitForTouchPressed();
		waitForTouchReleased();
	}

	/**
	 * Stalls execution unless the touch sensor is pressed down
	 */
	public static void waitForTouchPressed() { 
		float[] sample = new float[getTouchSensor().sampleSize()];
		// Wait for press
		while (true) {
			getTouchSensor().getTouchMode().fetchSample(sample, 0);
			if (sample[0] > 0.99)
				break;
		}
	}
	
	/**
	 * Stalls execution unless the touch sensor is not pressed down
	 */
	public static void waitForTouchReleased() {
		float[] sample = new float[getTouchSensor().sampleSize()];
		// Wait for released
		while (true) {
			getTouchSensor().getTouchMode().fetchSample(sample, 0);
			if (sample[0] < 0.1)
				break;
		}
	}
	
	private static RegulatedMotor getMotorA() {
		if (m_motorA == null)
			m_motorA = new EV3LargeRegulatedMotor(MotorPort.A);
		return m_motorA;
	}
	private static RegulatedMotor getMotorB() {
		if (m_motorB == null)
			m_motorB = new EV3LargeRegulatedMotor(MotorPort.B);
		return m_motorB;
	}
	private static RegulatedMotor getMotorC() {
		if (m_motorC == null)
			m_motorC = new EV3LargeRegulatedMotor(MotorPort.C);
		return m_motorC;
	}
	private static EV3TouchSensor getTouchSensor() {
		if (touchSensor == null) {
			touchSensor = new EV3TouchSensor(SensorPort.S1);
		}
		return touchSensor;
	}
	
}
