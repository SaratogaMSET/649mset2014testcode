/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends SimpleRobot {

    Victor pivot;
    Victor winch;
    Victor roller;
    Victor[] motors;
    Joystick shooterJoystick;
    Joystick driverLeftJoystick;
    Joystick driverRightJoystick;
    DoubleSolenoid winchSolenoid;
    DoubleSolenoid fingerSolenoid;
    DoubleSolenoid shifterSolenoid;
    DigitalInput limit;
    boolean fingerState = true;
    boolean fingerButtonState = false;
    Potentiometer potentiometer;
    Compressor compressor;

    protected void robotInit() {
        shooterJoystick = new Joystick(3);
        driverLeftJoystick = new Joystick(2);
        driverRightJoystick = new Joystick(1);
        roller = new Victor(7);
        winch = new Victor(5);
        pivot = new Victor(6);
        shifterSolenoid = new DoubleSolenoid(1, 2);
        winchSolenoid = new DoubleSolenoid(3, 4);
        fingerSolenoid = new DoubleSolenoid(5, 6);
        limit = new DigitalInput(9);
        motors = new Victor[4];
        for (int i = 0; i < 4; i++) {
            motors[i] = new Victor(i + 1);
        }
        compressor = new Compressor(1, 1);
        compressor.start();
        potentiometer = new AnalogPotentiometer(1);
    }

    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        fingerSolenoid.set(DoubleSolenoid.Value.kForward);
        while (isOperatorControl()) {
            Display.clear();
            getWatchdog().feed();
            driveFwdRot(getDriveForward(), getDriveRotation());
            shiftDriveGear(driverLeftJoystick.getRawButton(1) || driverRightJoystick.getRawButton(1));
            //button 5 to pull winch back
            if (shooterJoystick.getRawButton(5)) {
                winch.set(1);
                //test if starting and stopping the compressor again and again works
                compressor.stop();

            } else {
                winch.set(0);
                //especially check if its okay to keep calling the start command on the compressor every 20 millsecs
                compressor.start();
            }
            //button 1 to toggle winch sol (fire)
            if (shooterJoystick.getRawButton(1) && shooterJoystick.getRawButton(3)) {
                winchSolenoid.set(DoubleSolenoid.Value.kReverse);
            } else {
                winchSolenoid.set(DoubleSolenoid.Value.kForward);
            }
            final boolean rawButton = shooterJoystick.getRawButton(2);

            //button 2 to set finger to forward, 3 for off, default for forward
            if (rawButton && rawButton != fingerButtonState) {
                fingerState = !fingerState;
                fingerSolenoid.set(fingerState ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
            }
            fingerButtonState = rawButton;

            //button 4 for pickup, 6 for purge
            if (shooterJoystick.getRawButton(4)) {
                roller.set(-1);
            } else if (shooterJoystick.getRawButton(6)) {
                roller.set(1);
            } else {
                roller.set(0);
            }

            if (shooterJoystick.getY() < -.4) {
                pivot.set(.3);
            } else if (shooterJoystick.getY() > .4) {
                pivot.set(-.3);
            } else {
                pivot.set(0);
            }

            Display.queue("" + potentiometer.pidGet());
            Display.println(2, "" + potentiometer.pidGet());
            if (!compressor.getPressureSwitchValue()) {
                Display.println(3, "Running");
            }
            Display.println(4, limit.get() + "");
            Display.update();
            try {
                Thread.sleep(20);
            } catch (InterruptedException ex) {
                ex.printStackTrace();
            }

        }

    }

    public double getDriveForward() {
        return -driverLeftJoystick.getY();
    }

    public double getDriveRotation() {
        final double turnVal = driverRightJoystick.getX();
        final double sign = turnVal < 0 ? -1 : 1;
        return MathUtils.pow(Math.abs(turnVal), 1.4) * sign;
    }

    public void driveFwdRot(double fwd, double rot) {
        double left = fwd + rot, right = fwd - rot;
        double max = Math.max(1, Math.max(Math.abs(left), Math.abs(right)));
        left /= max;
        right /= max;
        rawDrive(left, right);
    }

    public void shiftDriveGear(boolean lowSpeed) {
        shifterSolenoid.set(lowSpeed ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    public void rawDrive(double left, double right) {
        int i = 0;

        for (; i < motors.length / 2; i++) {
            motors[i].set(left);
        }

        for (; i < motors.length; i++) {
            motors[i].set(-right);
        }
    }

    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {
    }
}
