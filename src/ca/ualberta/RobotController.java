package ca.ualberta;


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

	private static final int defaultDelay = 500;
	private static final int motor_speed = 60;
	private static final int motor_accel = 20;
	
	private static final double[] gear_ratios = {15d/30d, 1d};

	private static EV3TouchSensor touchSensor;
	
	public static void initializeMotorZero() {
		getLocation();
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
	public static void moveTo(int x, int y) {
		moveTo(new Point3D(x,y));
	}
	
	/**
	 * Moves the end effector to the point p
	 * @param p
	 */
	public static void moveTo(Point3D p) {
		System.out.println("Move to " + p);
		moveAnalytic(p);
	}
	
	/**
	 * Moves to start location then draws line from there to the final location
	 * @param start_location
	 * @param final_location
	 */
	public static void drawLineBW(Point3D start_location, Point3D final_location) {
		Point3D[] line = Kinematics.createLinePath(start_location, final_location);	
		for (Point3D target : line) {
			moveTo(target);
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
		
		motorA.rotateTo((int) (theta[0] / gear_ratios[0]),true);
		motorB.rotateTo((int) (theta[1] / gear_ratios[1]),false);	// last one should be false
		
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
