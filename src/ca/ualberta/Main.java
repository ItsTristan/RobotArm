package ca.ualberta;

import java.awt.Point;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;

public class Main {
	// A is the inner-most, B is the outer
		static RegulatedMotor A = new EV3LargeRegulatedMotor(MotorPort.A);
		static RegulatedMotor B = new EV3LargeRegulatedMotor(MotorPort.B);
		
		
	public static void main(String[] args) {
		//testFWDByHand();
		//print starting location
		Point p = Kinematics.forwardKinematics(new int[]{A.getTachoCount(), B.getTachoCount()});
		System.out.format("x = %d \ny= %d\n", p.x,p.y);
		
		A.rotateTo(90*2);
		B.rotateTo(90);
		Delay.msDelay(300);
		
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
