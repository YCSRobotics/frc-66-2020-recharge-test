package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileSystem;
import java.util.List;

public class Pathfollowing implements Loggable {
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
                    Constants.ksVolts,
                    Constants.kvVoltSecondsPerMeter,
                    Constants.kaVoltSecondsSquaredPerMeter);
    private Trajectory trajectory;
    private RamseteController controller = new RamseteController();

    private DifferentialDriveWheelSpeeds prevSpeeds;

    @Log.ToString
    private DifferentialDriveOdometry m_odometry;

    private Timer timer = new Timer();
    private double prevTime;

    @Log
    private PIDController leftPIDController = new PIDController(Constants.kPDriveVel, 0, 0);
    @Log
    private PIDController rightPIDController = new PIDController(Constants.kPDriveVel, 0, 0);

    @Log
    private double leftOutput;
    @Log
    private double rightOutput;

    @Log
    private double chassisSpeedsX;
    @Log
    private double chassisSpeedsY;
    @Log
    private double chassisSpeedsRadians;

    @Log
    private double curTime;

    public Pathfollowing() {
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("Center Start.wpilib.json"));
        } catch (IOException e){
            System.out.println("Error retrieving trajectory!");
            e.printStackTrace();
        }

        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
                Constants.kDriveKinematics,
                10
        );

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(SensorData.getYaw()));
    }

    public void initialize() {
        if (trajectory == null) {
            System.out.println("Trajectory is null!");
            return;
        }

        prevTime = 0;
        Trajectory.State initialState = trajectory.sample(0);

        prevSpeeds = Constants.kDriveKinematics.toWheelSpeeds(
                new ChassisSpeeds(initialState.velocityMetersPerSecond,
                        0,
                        initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));


        timer.reset();

        Drivetrain.resetEncoders();
        SensorData.resetAngle();

        SmartDashboard.putString("Starting Pose", trajectory.getInitialPose().toString());
        SmartDashboard.putNumber("Starting Yaw", SensorData.getYaw());

        m_odometry.resetPosition(trajectory.getInitialPose(), Rotation2d.fromDegrees(SensorData.getYaw()));

        leftPIDController.reset();
        rightPIDController.reset();

        timer.start();
    }

    public void followCenterPath() {
        if(trajectory == null) {
            return;
        }

        curTime = timer.get();
        double timeDifference = curTime - prevTime;

        m_odometry.update(Rotation2d.fromDegrees(SensorData.getYaw()), Drivetrain.getLeftDistanceMeters(), Drivetrain.getRightDistanceMeters());

        var speeds = controller.calculate(m_odometry.getPoseMeters(), trajectory.sample(curTime));

        chassisSpeedsX = speeds.vxMetersPerSecond;
        chassisSpeedsY = speeds.vyMetersPerSecond;
        chassisSpeedsRadians = speeds.omegaRadiansPerSecond;

        var targetWheelSpeeds = Constants.kDriveKinematics.toWheelSpeeds(speeds);

        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftFeedforward = feedforward.calculate(leftSpeedSetpoint, (leftSpeedSetpoint - prevSpeeds.leftMetersPerSecond) / timeDifference);
        double rightFeedforward = feedforward.calculate(rightSpeedSetpoint, (rightSpeedSetpoint - prevSpeeds.rightMetersPerSecond) / timeDifference);

        leftOutput = leftFeedforward + leftPIDController.calculate(Drivetrain.getWheelSpeeds().leftMetersPerSecond, leftSpeedSetpoint);
        rightOutput = rightFeedforward + rightPIDController.calculate(Drivetrain.getWheelSpeeds().rightMetersPerSecond, rightSpeedSetpoint);

        Drivetrain.setOutputVoltage(leftOutput, rightOutput);

        prevTime = curTime;
        prevSpeeds = targetWheelSpeeds;
    }

    public void updateOdometry() {
        m_odometry.update(Rotation2d.fromDegrees(SensorData.getYaw()), Drivetrain.getLeftDistanceMeters(), Drivetrain.getRightDistanceMeters());
    }
}
