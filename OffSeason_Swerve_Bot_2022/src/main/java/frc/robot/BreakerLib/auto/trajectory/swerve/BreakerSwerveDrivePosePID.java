// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.swerve;

import com.kauailabs.navx.IMUProtocol.YPRUpdate;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class BreakerSwerveDrivePosePID {
    private PIDController xPID, yPID, azimuthPID;
    private Pose2d lastSetpoint;

    public BreakerSwerveDrivePosePID(PIDController linearPID, PIDController azimuthPID) {
        xPID = linearPID;
        yPID = linearPID;
        this.azimuthPID = azimuthPID;
    }

    public ChassisSpeeds calculateFieldRelativeSpeeds(Pose2d measurement, Pose2d setpoint) {
        lastSetpoint = setpoint;
        double xSpeed = xPID.calculate(measurement.getX(), setpoint.getX());
        double ySpeed = yPID.calculate(measurement.getY(), setpoint.getY());
        double thetaSpeed = azimuthPID.calculate(measurement.getRotation().getRadians(), setpoint.getRotation().getRadians());
        return new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
    }

    public ChassisSpeeds calculateRobotRelatveSpeeds(Pose2d measurement, Pose2d setpoint) {
        ChassisSpeeds set = calculateFieldRelativeSpeeds(measurement, setpoint);
        return ChassisSpeeds.fromFieldRelativeSpeeds(set.vxMetersPerSecond, set.vyMetersPerSecond, set.omegaRadiansPerSecond, measurement.getRotation());
    }

    public Pose2d getSetpoint() {
        return lastSetpoint;
    }

    public boolean atSetpoint() {
        return xPID.atSetpoint() && yPID.atSetpoint() && azimuthPID.atSetpoint();
    }
}