package ca.ualberta;


import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
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
		moveNumerical(p);
	}
	
	/**
	 * Moves the end effector to the point p
	 * using an analytic solution
	 * @param p
	 */
	public static void moveAnalytic(Point3D p) {
		int[] theta = Kinematics.inverseKinematics(p);
		rotateTo(theta);
	}
	/**
	 * Moves the end effector to the point p
	 * using an analytic solution
	 * @param p
	 */
	public static void moveNumerical(Point3D p) {
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
		
		motorA.rotateTo((int) (theta[0] / gear_ratios[0]));
		motorB.rotateTo((int) (theta[1] / gear_ratios[1]));
		
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
}
