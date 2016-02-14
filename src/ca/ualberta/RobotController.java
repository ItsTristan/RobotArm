package ca.ualberta;


import java.io.FileNotFoundException;
import java.io.PrintWriter;

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

	private static final int defaultDelay = 100;
	private static final int motor_speed = 60;
	private static final int motor_accel = 20;
	
	private static final double[] gear_ratios = {15d/30d, 1d, -1d};

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
	
	public static void resetMotors() {
		System.out.println("Resetting...");
		rotateTo(new int[] {0,0,0}, 300);
		relaxMotors();
		getMotorA().close();
		m_motorA = null;
		
		getMotorB().close();
		m_motorB = null;
		
		getMotorC().close();
		m_motorC = null;
	}
	/***
	 * Moves the end effector to the point (x,y)
	 * @param x
	 * @param y
	 */
	public static void moveTo(double x, double y) {
		moveAnalytic(new Point3D(x,y));
	}

	/**
	 * Moves the end effector to the point p
	 * @param p
	 */
	public static void moveTo(Point3D p) {
		//System.out.println("Move to " + p);
		moveNumerical(p);
	}
	
	/**
	 * Moves the end effector to the point p
	 * @param p
	 */
	public static void moveTo3D(Point3D p) {
		//System.out.println("Move to " + p);
		System.out.println("Moving in 3D");
		moveNumerical3D(p);
	}
	
	/**
	 * Moves to start location then draws line from there to the final location
	 * @param start_location
	 * @param final_location
	 */
	public static void drawLineBW(Point3D start, Point3D end) {
		System.out.format("start: " + start + "\nend: "+end+"\n");
		Point3D[] line = Kinematics.createLinePath(start, end);	
		for (Point3D target : line) {
			moveTo(target);
		}
	}
	/**
	 * Given 2 end points of an arc and any other point on the arc can step 
	 * along and draw the entire arc.
	 * @param start 
	 * @param mid is any point between start and end that lies on the arc
	 * @param end
	 */
	public static void drawArcLine(Point3D start, Point3D mid, Point3D end){
		Point3D center = Kinematics.getCenterArc(start, mid, end);
		double radius = center.distance(start);
		double arc_angle = Math.toRadians(Kinematics.getAngleBWlines(center, start, end));
		double step_size = arc_angle/20; 
		moveTo(start);
		
		//DEBUG
		String filename = "arc_out.txt";
		PrintWriter out = null;
		try {
			out = new PrintWriter(filename);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
						
		double start_angle = Math.atan2(start.y - center.y, start.x - center.x);
		out.printf("\nstart: %.2f,%.2f\n", start.x,start.y); //debug
		for (int i = 0; i < 20; i++){
			double next_x = radius*Math.cos(start_angle-step_size*i) + center.x;
			double next_y = radius*Math.sin(start_angle-step_size*i) + center.y;
			out.printf("\nnext: %.2f,%.2f\n",  next_x, next_y); //debug
			moveTo( next_x, next_y);
		}
		out.close();
		moveTo(end);
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
		int[] theta = Kinematics.inverseKinematics(p, getJointAngles());
		rotateTo(theta);
	}
	/**
	 * Moves the end effector to the point p
	 * using an analytic solution
	 * @param p
	 */
	public static void moveNumerical3D(Point3D p) {
		int[] theta = Kinematics.inverseNumericalKinematics(p, getJointAngles3());
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
		RegulatedMotor motorC = getMotorC();

		motorA.setAcceleration(motor_accel);
		motorB.setAcceleration(motor_accel);
		motorC.setAcceleration(motor_accel);

		motorA.setSpeed(motor_speed);
		motorB.setSpeed(motor_speed);
		motorC.setSpeed(motor_speed);
		
		for (int i = 0; i < theta.length; i++) {
			if (theta[i] > 180) {
				theta[i] -= 360;
			} else if (theta[i] < -180) {
				theta[i] += 360;
			}
		}

		motorA.rotateTo((int) ( theta[0] / gear_ratios[0]),true);
		motorB.rotateTo((int) ( theta[1] / gear_ratios[1]),false);	// last one should be false
		
		if (theta.length == 2) {
			motorC.rotateTo(0, false);
		} else {
			motorC.rotateTo((int) (theta[2] / gear_ratios[2]), false);
		}
		
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
	
	public static Point3D getLocation3() {
		return Kinematics.forwardKinematics(getJointAngles3());
	}

	/**
	 * Returns the current joint angles, accounting for the gear ratios.
	 * @return
	 */
	public static int[] getJointAngles() {
		return new int[] {
				(int) (getMotorA().getTachoCount()*gear_ratios[0]),
				(int) (getMotorB().getTachoCount()*gear_ratios[1]),
		};
	}
	/**
	 * Returns the current joint angles, accounting for the gear ratios.
	 * @return
	 */
	public static int[] getJointAngles3() {
		return new int[] {
				(int) (getMotorA().getTachoCount()*gear_ratios[0]),
				(int) (getMotorB().getTachoCount()*gear_ratios[1]),
				(int) (getMotorC().getTachoCount()*gear_ratios[2]),
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
