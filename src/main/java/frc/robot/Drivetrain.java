/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

/**
 * Handles robot horizontal and vertical movement
 */
public class Drivetrain {
    private static WPI_TalonSRX motorLeftMaster = new WPI_TalonSRX(Constants.kMotorLeftMasterPort);
    private static WPI_TalonSRX motorLeftFollower = new WPI_TalonSRX(Constants.kMotorLeftFollowerPort);
    private static WPI_TalonSRX motorRightMaster = new WPI_TalonSRX(Constants.kMotorRightMasterPort);
    private static WPI_TalonSRX motorRightFollower = new WPI_TalonSRX(Constants.kMotorRightFollowerPort);

    private DifferentialDrive diffyDrive = new DifferentialDrive(motorLeftMaster, motorRightMaster);

    private Joystick driverJoy = JoystickOI.driverJoy;
    private Joystick operatorJoy = JoystickOI.operatorJoy;

    public Drivetrain() {
        // only initialization values should be placed here
        motorLeftFollower.set(ControlMode.Follower, motorLeftMaster.getDeviceID());
        motorRightFollower.set(ControlMode.Follower, motorRightMaster.getDeviceID());

        // ctre examples had these, remove if causing problems
        motorRightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        motorLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        // invert right sensor values
        motorRightMaster.setSensorPhase(true);

        motorLeftMaster.setSelectedSensorPosition(0, 0, 10);
        motorRightMaster.setSelectedSensorPosition(0, 0, 10);
    }

    /**
     * Handles periodic robot control of the drivetrain
     */
    public void drivetrainPeriodic() {
        double throttle = driverJoy.getRawAxis(XboxController.Axis.kLeftY.value);
        double turn = driverJoy.getRawAxis(XboxController.Axis.kRightX.value);

        diffyDrive.arcadeDrive(throttle, turn, true);

        //feed motor safety
        motorLeftMaster.feed();
        motorRightMaster.feed();
    }

    public static void resetEncoders() {
        motorLeftMaster.setSelectedSensorPosition(0, 0, 10);
        motorRightMaster.setSelectedSensorPosition(0, 0, 10);
    }

    public static double getLeftDistanceMeters() {
        return motorLeftMaster.getSelectedSensorPosition() / Constants.kTotalEdgesPerRevolution;
    }

    public static double getRightDistanceMeters() {
        return motorRightMaster.getSelectedSensorPosition() / Constants.kTotalEdgesPerRevolution;
    }

    public static DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                motorLeftMaster.getSelectedSensorVelocity() / Constants.kTotalEdgesPerRevolution,
                motorRightMaster.getSelectedSensorVelocity() / Constants.kTotalEdgesPerRevolution
                );
    }

    /**
     * Be warned of setting robot output in multiple locations
     * @param leftVoltage
     * @param rightVoltage
     */
    public static void setOutputVoltage(double leftVoltage, double rightVoltage) {
        var leftOutput = leftVoltage / Constants.kMaxBatVoltage;
        var rightOutput = rightVoltage / Constants.kMaxBatVoltage;

        motorLeftMaster.set(ControlMode.PercentOutput, leftOutput);
        motorRightMaster.set(ControlMode.PercentOutput, rightOutput);

        motorLeftMaster.feed();
        motorRightMaster.feed();
    }

    public static void configureVoltageCompensation() {
        motorLeftMaster.configVoltageCompSaturation(Constants.kMaxBatVoltage);
        motorLeftMaster.enableVoltageCompensation(true);

        motorRightMaster.configVoltageCompSaturation(Constants.kMaxBatVoltage);
        motorRightMaster.enableVoltageCompensation(true);
    }
}
