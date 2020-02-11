/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;



import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Grizzly Robotics Lift Class Handles controlling the robot fourbar
 */
public class Shooter {
    /**
     * deviceID is the CAN ID of the SPARK MAX you are using. Change to match your
     * setup
     */
    private static CANSparkMax shooterMotorMaster = new CANSparkMax(Constants.kNeoDeviceID, MotorType.kBrushless);

    private Joystick operatorController = JoystickOI.operatorJoy;

    // private double fourBarPosition = 0.0;
    // public static double setPosition = 0.0;

    // private static boolean override = false;

    public Shooter() {
        // Configure Neo 550
        /**
         * The restoreFactoryDefaults method can be used to reset the configuration
         * parameters in the SPARK MAX to their factory default state. If no argument is
         * passed, these parameters will not persist between power cycles
         */
        shooterMotorMaster.restoreFactoryDefaults();

        /**
         * Parameters can be set by calling the appropriate Set method on the
         * CANSparkMax object whose properties you want to change
         * 
         * Set methods will return one of three CANError values which will let you know
         * if the parameter was successfully set: CANError.kOk CANError.kError
         * CANError.kTimeout
         */
        if (shooterMotorMaster.setIdleMode(IdleMode.kCoast) != CANError.kOk) {
            SmartDashboard.putString("Idle Mode", "Error");
        }

        /**
         * Similarly, parameters will have a Get method which allows you to retrieve
         * their values from the controller
         */
        if (shooterMotorMaster.getIdleMode() == IdleMode.kCoast) {
            SmartDashboard.putString("Idle Mode", "Coast");
        } else {
            SmartDashboard.putString("Idle Mode", "Brake");
        }

        // Set ramp rate to 0
        if (shooterMotorMaster.setOpenLoopRampRate(0) != CANError.kOk) {
            SmartDashboard.putString("Ramp Rate", "Error");
        }

        // read back ramp rate value
        SmartDashboard.putNumber("Ramp Rate", shooterMotorMaster.getOpenLoopRampRate());
    }

    /**
     * Periodic method to update shooter
     */
    public void updateShooter() {
        // Set motor output to joystick value
        shooterMotorMaster.set(operatorController.getY());

        // periodically read voltage, temperature, and applied output and publish to
        // SmartDashboard
        SmartDashboard.putNumber("Voltage", shooterMotorMaster.getBusVoltage());
        SmartDashboard.putNumber("Temperature", shooterMotorMaster.getMotorTemperature());
        SmartDashboard.putNumber("Output", shooterMotorMaster.getAppliedOutput());

    }

    // public static double getFourBarPosition() {
    //     return fourBarMotorMaster.getSelectedSensorPosition(0);
    // }

    // public static void setOverride(boolean state) {
    //     override = state;
    // }

}
