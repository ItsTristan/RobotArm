package ca.ualberta;

import java.awt.Point;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;

public class RobotController {

	private static RegulatedMotor m_motorA;
	private static RegulatedMotor m_motorB;
	private static RegulatedMotor m_motorC;

	public static final int motor_speed = 60;
	public static final int motor_accel = 20;
	
	public static final double[] gear_ratios = {15d/30d, 1d};
	
	/***
	 * Moves the end effector to the point (x,y)
	 * @param x
	 * @param y
	 */
	public static void moveTo(int x, int y) {
		moveTo(new Point(x,y));
	}
	
	/**
	 * Moves the end effector to the point p
	 * @param p
	 */
	public static void moveTo(Point p) {
		return;
	}
	
	/**
	 * Rotates the joints to the given angles, returning
	 * the point of the end effector
	 * @param theta
	 * @return
	 */
	public static void rotateTo(int[] theta) {
		RegulatedMotor motorA = getMotorA();
		RegulatedMotor motorB = getMotorB();	

		motorA.setAcceleration(motor_accel);
		motorB.setAcceleration(motor_accel);

		motorA.setSpeed(motor_speed);
		motorB.setSpeed(motor_speed);
		
		motorA.rotateTo((int) (theta[0] / gear_ratios[0]));
		motorB.rotateTo((int) (theta[1] / gear_ratios[1]));
		
		Delay.msDelay(300);
	}
	
	/**
	 * Returns the current location of the end effector,
	 * accounting for the gear ratios.
	 * @return
	 */
	public static Point getLocation() {
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
