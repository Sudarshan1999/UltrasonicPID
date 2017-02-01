package org.usfirst.frc.team1165.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends SampleRobot
{
    // distance in inches the robot wants to stay from an object
    private static final double kHoldDistance = 4;

    // maximun distance in inches we expect the robot to see
    private static final double kMaxDistance = 15.0;

    // factor to convert sensor values to a distance in inches
    private static final double kValueToInches = 0.125;

    // proportional speed constant
    private static final double kP = 7.0;

    // integral speed constant
    private static final double kI = 0.018;

    // derivative speed constant
    private static final double kD = 1.5;

    Joystick stick = new Joystick(0);
    JoystickButton enableUltrasonic = new JoystickButton(stick, 10);

    RobotDrive robotDrive;

    // Channels for the wheels
    CANTalon frontLeft = new CANTalon(0);
    CANTalon rearLeft = new CANTalon(1);
    CANTalon frontRight = new CANTalon(2);
    CANTalon rearRight = new CANTalon(3);

    private static final int ultrasonicLeftPingChannel = 0;
    private static final int ultrasonicLeftEchoChannel = 1;
    private static final int ultrasonicRightPingChannel = 2;
    private static final int ultrasonicRightEchoChannel = 3;

    private Ultrasonic ultrasonicLeft;
    private Ultrasonic ultrasonicRight;

    private PIDController pidController;
    private static boolean finished = false;

    public Robot()
    {

	ultrasonicLeft = new Ultrasonic(ultrasonicLeftPingChannel, ultrasonicLeftEchoChannel);
	ultrasonicRight = new Ultrasonic(ultrasonicRightPingChannel, ultrasonicRightEchoChannel);
	ultrasonicLeft.setDistanceUnits(Unit.kInches);
	ultrasonicRight.setDistanceUnits(Unit.kInches);
	ultrasonicLeft.setAutomaticMode(true);
	ultrasonicRight.setAutomaticMode(true);

	robotDrive = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
	robotDrive.setInvertedMotor(MotorType.kFrontLeft, true); // invert the
								 // left side
								 // motors
	robotDrive.setInvertedMotor(MotorType.kRearLeft, true); // you may need
								// to change or
								// remove this
								// to match your
								// robot
	robotDrive.setExpiration(0.1);
	if (ultrasonicLeft.getRangeInches() < kHoldDistance)
	{
	    if (ultrasonicRight.getRangeInches() < kHoldDistance)
	    {
		// Neither sees the boiler opening. You are ready to shoot.
		pidController = new PIDController(kP, kI, kD, ultrasonicLeft, new MyPidOutput());
		finished = true;
	    } else
	    {
		// Left does not see boiler opening but right does. Strafe
		// right.
		pidController = new PIDController(kP, kI, kD, ultrasonicRight, new MyPidOutput());
		finished = false;
	    }
	} else
	{
	    if (ultrasonicRight.getRangeInches() < kHoldDistance)
	    {
		// Left sees the boiler opening but right does not. Strafe Left
		pidController = new PIDController(kP, kI, kD, ultrasonicLeft, new MyPidOutput());
		finished = false;
	    } else
	    {
		// Both controllers see greater than kHoldDistance inches; you
		// are clearly not near the boiler
		// Get in position and try again.
		pidController = new PIDController(kP, kI, kD, ultrasonicRight, new MyPidOutput());
		finished = true;
	    }
	}
	// Set expected range to 2-24 inches; e.g. at 24 inches from object
	// strafe
	// left or right, at 2 inches from object stop.
	pidController.setInputRange(0, kMaxDistance);
	// Set setpoint of the pidController
	pidController.setSetpoint(kHoldDistance);
    }

    /**
     * Runs the motors with Mecanum drive.
     */
    @Override
    public void operatorControl()
    {
	robotDrive.setSafetyEnabled(true);
	while (isOperatorControl() && isEnabled())
	{
	    double x = stick.getX() * stick.getX() * stick.getX();
	    double y = stick.getY() * stick.getY() * stick.getY();
	    double twist = stick.getTwist() * stick.getTwist() * stick.getTwist();
	    if (Math.abs(x) < 0.1)
		x = 0;
	    if (Math.abs(y) < 0.1)
		y = 0;
	    if (Math.abs(twist) < 0.1)
		twist = 0;
	    
	    //Ultrasonic
		if (ultrasonicLeft.getRangeInches() < kHoldDistance)
		{
		    if (ultrasonicRight.getRangeInches() < kHoldDistance)
		    {
			// Neither sees the boiler opening. You are ready to shoot.
			pidController = new PIDController(kP, kI, kD, ultrasonicLeft, new MyPidOutput());
			finished = true;
		    } else
		    {
			// Left does not see boiler opening but right does. Strafe
			// right.
			pidController = new PIDController(kP, kI, kD, ultrasonicLeft, new MyPidOutput());
			finished = false;
		    }
		} else
		{
		    if (ultrasonicRight.getRangeInches() < kHoldDistance)
		    {
			// Left sees the boiler opening but right does not. Strafe Left
			pidController = new PIDController(kP, kI, kD, ultrasonicRight, new MyPidOutput());
			finished = false;
		    } else
		    {
			// Both controllers see greater than kHoldDistance inches
			// You are clearly not near the boiler
			// Get in position and try again.
			pidController = new PIDController(kP, kI, kD, ultrasonicRight, new MyPidOutput());
			finished = true;
		    }
		}
		// Set expected range to 2-24 inches; e.g. at 24 inches from object
		// strafe
		// left or right, at 2 inches from object stop.
		pidController.setInputRange(0, kMaxDistance);
		// Set setpoint of the pidController
		pidController.setSetpoint(kHoldDistance);
	    // Use the joystick X axis for lateral movement, Y axis for forward
	    // movement, and Z axis for rotation.
	    // This sample does not use field-oriented drive, so the gyro input
	    // is set to zero.
	    if (enableUltrasonic.get())
	    {
		pidController.enable();
	    }
	    SmartDashboard.putNumber("Ultrasonic Left", ultrasonicLeft.getRangeInches());
	    SmartDashboard.putNumber("Ultrasonic Right", ultrasonicRight.getRangeInches());

	    robotDrive.mecanumDrive_Cartesian(x, y, twist, 0);

	    Timer.delay(0.005); // wait 5ms to avoid hogging CPU cycles
	}
    }

    private class MyPidOutput implements PIDOutput
    {
	@Override
	public void pidWrite(double output)
	{
	    if (!finished)
		robotDrive.mecanumDrive_Cartesian(output, 0, 0, 0);
	}
    }
}
