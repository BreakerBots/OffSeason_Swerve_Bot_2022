// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.BreakerLib.physics.vector.BreakerVector3;

/** Add your docs here. */
public class BreakerTurretState {
    private Rotation2d azimuthAngle, altitudeAngle;
    private double flywheelRPM;
    public BreakerTurretState(Rotation2d azimuthAngle, Rotation2d altitudeAngle, double flywheelRPM) {
        this.altitudeAngle = altitudeAngle;
        this.azimuthAngle = azimuthAngle;
        this.flywheelRPM = flywheelRPM;
    }

    public BreakerTurretState toRobotRelativeState(Rotation3d robotRot) {
        return new BreakerTurretState(azimuthAngle.plus(robotRot.toRotation2d()), altitudeAngle.plus(new Rotation2d(robotRot.getY())), flywheelRPM);
    }
}
