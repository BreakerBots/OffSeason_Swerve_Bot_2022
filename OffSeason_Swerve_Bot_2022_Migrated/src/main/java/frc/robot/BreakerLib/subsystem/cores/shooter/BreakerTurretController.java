// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.shooter;

import java.util.function.Consumer;
import java.util.function.Supplier;

import javax.naming.spi.DirStateFactory;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.BreakerLib.physics.projectilemotion.BreakerProjectile;
import frc.robot.BreakerLib.physics.projectilemotion.BreakerProjectileTrajectory;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;
import frc.robot.BreakerLib.physics.vector.BreakerVector3;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolableDouble;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolablePair;
import frc.robot.BreakerLib.util.math.interpolation.interpolateingmaps.BreakerInterpolatingTreeMap;

/** Add your docs here. */
public class BreakerTurretController {
    private BreakerInterpolatingTreeMap<Double,BreakerVector2> fireingTable;
    private BreakerInterpolatingTreeMap<Double, BreakerInterpolableDouble> rpmToProjectileLaunchVelocity;
    private BreakerProjectile projectile;
    /** @param projectile
     *  @param fireingTable fireing distance compared to BreakerVector2(angle and RPM)
     *  @param rpmToProjectileLaunchVelocity interpolating table that relates flywheel RPM to the launch velocity of the projectile
      */
    public BreakerTurretController(BreakerProjectile projectile, BreakerInterpolatingTreeMap<Double,BreakerVector2> fireingTable, BreakerInterpolatingTreeMap<Double, BreakerInterpolableDouble> rpmToProjectileLaunchVelocity) {

    }

    public BreakerTurretState calculateFieldRelative(Translation3d projectileLaunchPointRelativeToField, Translation3d targetPointRelativeToField) {
        double distance = projectileLaunchPointRelativeToField.toTranslation2d().getDistance(targetPointRelativeToField.toTranslation2d());
        BreakerVector2 fireingSolution = fireingTable.getInterpolatedValue(distance);

        Rotation2d azAng = BreakerMath.getPointAngleRelativeToOtherPoint(projectileLaunchPointRelativeToField.toTranslation2d(), targetPointRelativeToField.toTranslation2d());
        Rotation2d altAng = fireingSolution.getVectorRotation();

       return new BreakerTurretState(azAng, altAng, fireingSolution.getMagnatude());
    }

    public BreakerTurretState calculateRobotRelative(Pose3d projectileLaunchPointRelativeToField, Translation3d targetPointRelativeToField) {
        BreakerTurretState calcState = calculateFieldRelative(projectileLaunchPointRelativeToField.getTranslation(), targetPointRelativeToField);
        return calcState.toRobotRelativeState(projectileLaunchPointRelativeToField.getRotation());
    }
}