/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * Robot-wide "public static final" constants
 * Should be in kHungarian notation
 */
public class Constants {
    // Motor constants
    public static final int kMotorLeftMasterPort = 0;
    public static final int kMotorLeftFollowerPort = 1;
    public static final int kMotorRightMasterPort = 2;
    public static final int kMotorRightFollowerPort = 3;

    public static final int kMaxBatVoltage = 12;
    // edges per revolution
    // includes gear ratio
    public static final double kTotalEdgesPerRevolution = 19516;

    // characterization constants, all in meters
    public static final double ksVolts = 1.37;
    public static final double kvVoltSecondsPerMeter = 1.68;
    public static final double kaVoltSecondsSquaredPerMeter = 0.679;

    //public static final double kPDriveVel = 20.2;
    public static final double kPDriveVel = 1;

    public static final double kTrackwidthMeters = 0.65;
    public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

    // how fast we want to follow and accelerate during the trajectory
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecond = 3;

    // ramsete constants, don't change
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
