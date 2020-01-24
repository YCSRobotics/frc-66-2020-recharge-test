package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

import java.io.IOException;

public class Pathfollowing {
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
                    Constants.ksVolts,
                    Constants.kvVoltSecondsPerMeter,
                    Constants.kaVoltSecondsSquaredPerMeter);
    private Trajectory trajectory;
    private RamseteController controller = new RamseteController();
    private DifferentialDriveWheelSpeeds prevSpeeds;

    private DifferentialDriveOdometry m_odometry;

    private Timer timer = new Timer();
    private double prevTime;

    private PIDController leftPIDController = new PIDController(Constants.kPDriveVel, 0, 0);
    private PIDController rightPIDController = new PIDController(Constants.kPDriveVel, 0, 0);

    public Pathfollowing() {
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("Center Start.wpilib.json"));
        } catch (IOException e){
            System.out.println("Error retrieving trajectory!");
            e.printStackTrace();
        }
    }

    public void initialize() {
        prevTime = 0;
        Trajectory.State initialState = trajectory.sample(0);

        prevSpeeds = Constants.kDriveKinematics.toWheelSpeeds(
                new ChassisSpeeds(initialState.velocityMetersPerSecond,
                        0,
                        initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));


        timer.reset();
        timer.start();

        SensorData.resetAngle();

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(SensorData.getYaw()));
        leftPIDController.reset();
        rightPIDController.reset();
    }

    public void followCenterPath() {
        if(trajectory == null) {
            System.out.println("Trajectory is null!");
            return;
        }

        double curTime = timer.get();
        double timeDifference = curTime - prevTime;

        m_odometry.update(Rotation2d.fromDegrees(SensorData.getYaw()), Drivetrain.getLeftDistanceMeters(), Drivetrain.getRightDistanceMeters());

        var targetWheelSpeeds = Constants.kDriveKinematics.toWheelSpeeds(
               controller.calculate(m_odometry.getPoseMeters(), trajectory.sample(curTime))
        );

        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftFeedforward = feedforward.calculate(leftSpeedSetpoint, (leftSpeedSetpoint - prevSpeeds.leftMetersPerSecond) / timeDifference);
        double rightFeedforward = feedforward.calculate(rightSpeedSetpoint, (rightSpeedSetpoint - prevSpeeds.rightMetersPerSecond) / timeDifference);

        var leftOutput = leftFeedforward + leftPIDController.calculate(Drivetrain.getWheelSpeeds().leftMetersPerSecond, leftSpeedSetpoint);
        var rightOutput = rightFeedforward + rightPIDController.calculate(Drivetrain.getWheelSpeeds().rightMetersPerSecond, rightSpeedSetpoint);

        Drivetrain.setOutputVoltage(leftOutput, rightOutput);

        prevTime = curTime;
        prevSpeeds = targetWheelSpeeds;
    }

    public void updateOdometry() {
        m_odometry.update(Rotation2d.fromDegrees(SensorData.getYaw()), Drivetrain.getLeftDistanceMeters(), Drivetrain.getRightDistanceMeters());
    }
}
