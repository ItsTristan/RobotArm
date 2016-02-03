package ca.ualberta;

import java.awt.Point;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;

public class Main {
	// A is the inner-most, B is the outer
		static RegulatedMotor A = new EV3LargeRegulatedMotor(MotorPort.A);
		static RegulatedMotor B = new EV3LargeRegulatedMotor(MotorPort.B);
		private static EV3TouchSensor touchSensor;
		
	public static void main(String[] args) {
		// maybe make menu on brick??
			//DEBUG GET ANGLE IN KINEMATICS
		doAngleBWlines(); //NEED TO TEST more
		//getDistance(); //NEED TO TEST
		//testFWDByHand(); //NEED TO TEST and make use sensor
		//doForward2DbyAngle(90, 90);
	}
	
	private static int getDistance(){
		Point[] points = new Point[2];
		int pnum = 0;
		if (touchSensor == null) {
			touchSensor = new EV3TouchSensor(SensorPort.S1);
		}
		float[] sample = new float[touchSensor.sampleSize()];
		while (pnum <= 2){
			touchSensor.getTouchMode().fetchSample(sample, 0);  
			if (sample[0] == 1){
				points[pnum] = Kinematics.forwardKinematics(new int[]{A.getTachoCount(), B.getTachoCount()});
				pnum++;
			}
		}
		int distance = (int) Math.sqrt((Math.pow(points[0].x - points[1].x, 2)
						+ Math.pow(points[0].y - points[1].y, 2)));
		System.out.format("point1= (%d,%d) \npoint2= (%d,%d) \ndistance: %d", 
						points[0].x, points[0].y, points[1].x, points[1].y, distance);
		Button.waitForAnyPress();
		return distance;
	}
	
	private static void doAngleBWlines() {
		Point[] cba = new Point[3];
		int points = 0;
		if (touchSensor == null) {
			touchSensor = new EV3TouchSensor(SensorPort.S1);
		}
		float[] sample = new float[touchSensor.sampleSize()];
		while (points <= 2){
			touchSensor.getTouchMode().fetchSample(sample, 0); 
			//System.out.format("sample= %.2f\n", sample[0]);
			if (sample[0] == 1.0){
				cba[points] = Kinematics.forwardKinematics(new int[]{A.getTachoCount(), B.getTachoCount()});
				System.out.format("point%d= %d,%d\n", points, cba[points].x, cba[points].y);
				points++;
			}
			Delay.msDelay(500);
		}
		int angle = Kinematics.getAngleBWlines(cba[0], cba[1], cba[2]);
		System.out.format("inter ang= %d \n", angle);
		Button.waitForAnyPress();
	}

	private static void doForward2DbyAngle(int angleA, int angleB) {
		//print starting location
		Point p = Kinematics.forwardKinematics(new int[]{A.getTachoCount(), B.getTachoCount()});
		System.out.format("x = %d \ny= %d\n", p.x,p.y);
		angleA = 2*angleA; // for use of gears
		A.rotateTo(angleA);
		B.rotateTo(angleB);
		Delay.msDelay(200);
		
		int At = A.getTachoCount();
		int Bt = B.getTachoCount();
		p = Kinematics.forwardKinematics(new int[]{At, Bt});
		System.out.format("x = %d \ny= %d \nTachoA: %d \nTachoB: %d\n", p.x,p.y, At,Bt );
		Button.waitForAnyPress();
	}

	private static void testFWDByHand() {
		while (true) {
			Point p = Kinematics.forwardKinematics(new int[]{A.getTachoCount(), B.getTachoCount()});
			System.out.format("x = %d \ny= %d\n", p.x,p.y);		
			Button.waitForAnyPress();	
		}
	}

	public static boolean check_fields(int source, int flag) {
		return (source & flag) != 0;
	}
}
