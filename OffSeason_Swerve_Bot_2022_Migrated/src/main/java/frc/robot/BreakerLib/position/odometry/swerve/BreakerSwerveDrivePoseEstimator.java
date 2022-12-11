// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.swerve;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.util.math.BreakerMath;
/**
 * The generic arguments to this class define the size of the state, input and output vectors used in the underlying Unscented Kalman Filter. 
 * States must be equal to the module count + 3. Inputs must be equal to the module count + 3. Outputs must be equal to the module count + 1.
 */
public class BreakerSwerveDrivePoseEstimator<States extends Num, Inputs extends Num, Outputs extends Num> extends SubsystemBase implements BreakerGenericOdometer  {
    private BreakerGenericGyro gyro;
    private SwerveDrivePoseEstimator<States, Inputs, Outputs> poseEstimator;
    private BreakerSwerveDrive drivetrain;
    private double lastUpdateTimestamp = Timer.getFPGATimestamp();
    private Pose2d prevPose = getOdometryPoseMeters();
    private ChassisSpeeds fieldRelativeChassisSpeeds = new ChassisSpeeds();
    private BreakerMovementState2d prevMovementState = new BreakerMovementState2d();
    private BreakerMovementState2d curMovementState = new BreakerMovementState2d();

    public BreakerSwerveDrivePoseEstimator(
        BreakerSwerveDrive drivetrain,
        Pose2d initialPose, 
        Nat<States> states,
        Nat<Inputs> inputs,
        Nat<Outputs> outputs,
        double[] stateModelStanderdDeveation, 
        double[] gyroAndEncoderStandardDeveation, 
        double[] visionStanderdDeveation
    ) {
        this.drivetrain = drivetrain;
        poseEstimator = new SwerveDrivePoseEstimator<States, Inputs, Outputs>(
            states, inputs, outputs, gyro.getYawRotation2d(), 
            initialPose, drivetrain.getSwerveModulePositions(),
                drivetrain.getConfig().getKinematics(),
                new MatBuilder<>(states, Nat.N1()).fill(stateModelStanderdDeveation),
                new MatBuilder<>(outputs, Nat.N1()).fill(gyroAndEncoderStandardDeveation),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(visionStanderdDeveation));
    }

    public Pose2d addVisionMeasurment(Pose2d robotPoseFromVision, double visionDataTimestamp) {
        poseEstimator.addVisionMeasurement(robotPoseFromVision,
                visionDataTimestamp);
        return poseEstimator.getEstimatedPosition();
    }

    public void changeVisionDevs(double visionStrdDevX, double visionStrdDevY, double visionStrdDevTheta) {
        poseEstimator.setVisionMeasurementStdDevs(
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(visionStrdDevX, visionStrdDevY, visionStrdDevTheta));
    }

    @Override
    public String toString() {
        return "CURRENT POSE: " + getOdometryPoseMeters().toString();
    }

    @Override
    public void setOdometryPosition(Pose2d newPose) {
        poseEstimator.resetPosition(newPose, gyro.getYawRotation2d());
    }

    @Override
    public Pose2d getOdometryPoseMeters() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public BreakerMovementState2d getMovementState() {
        return curMovementState;
    }

    @Override
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeChassisSpeeds.vxMetersPerSecond, 
            fieldRelativeChassisSpeeds.vyMetersPerSecond, 
            fieldRelativeChassisSpeeds.omegaRadiansPerSecond, 
            getOdometryPoseMeters().getRotation());
    }

    @Override
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return fieldRelativeChassisSpeeds;
    }

    private void updateChassisSpeeds() {
        double timeDiff = Timer.getFPGATimestamp() - lastUpdateTimestamp;
        double xSpeed = (getOdometryPoseMeters().getX() - prevPose.getX()) * timeDiff;
        double ySpeed = (getOdometryPoseMeters().getY() - prevPose.getY()) * timeDiff;
        double thetaSpeed = (getOdometryPoseMeters().getRotation().getRadians() - prevPose.getRotation().getRadians()) * timeDiff;
        fieldRelativeChassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        curMovementState = BreakerMath.movementStateFromChassisSpeedsAndPreviousState(getOdometryPoseMeters(), fieldRelativeChassisSpeeds, timeDiff, prevMovementState);
    }

    @Override
    public void periodic() {
        prevPose = getOdometryPoseMeters();
        poseEstimator.update(gyro.getYawRotation2d(), drivetrain.getSwerveModuleStates(), drivetrain.getSwerveModulePositions());
        updateChassisSpeeds();
        lastUpdateTimestamp = Timer.getFPGATimestamp();
    }

}
