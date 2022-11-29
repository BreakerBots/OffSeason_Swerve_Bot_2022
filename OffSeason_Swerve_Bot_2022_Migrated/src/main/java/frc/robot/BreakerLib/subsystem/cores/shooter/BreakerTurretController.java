// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.BreakerLib.physics.projectile.BreakerProjectile;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolableDouble;
import frc.robot.BreakerLib.util.math.interpolation.interpolateingmaps.BreakerInterpolatingTreeMap;

/** Robot turret system. */
public class BreakerTurretController {

    private BreakerInterpolatingTreeMap<Double,BreakerVector2> firingTable;
    private BreakerInterpolatingTreeMap<Double, BreakerInterpolableDouble> rpmToProjectileLaunchVelocity;
    private BreakerProjectile projectile;

    /** @param projectile
     *  @param firingTable Firing distance compared to BreakerVector2(angle and RPM)
     *  @param rpmToProjectileLaunchVelocity Interpolating table that relates flywheel RPM to the launch velocity of the projectile.
      */
    public BreakerTurretController(BreakerProjectile projectile, BreakerInterpolatingTreeMap<Double,BreakerVector2> firingTable, BreakerInterpolatingTreeMap<Double, BreakerInterpolableDouble> rpmToProjectileLaunchVelocity) {

    }

    public BreakerTurretState calculateFieldRelative(Translation3d projectileLaunchPointRelativeToField, Translation3d targetPointRelativeToField) {
        double distance = projectileLaunchPointRelativeToField.toTranslation2d().getDistance(targetPointRelativeToField.toTranslation2d());
        BreakerVector2 firingSolution = firingTable.getInterpolatedValue(distance);

        Rotation2d azAng = BreakerMath.getPointAngleRelativeToOtherPoint(projectileLaunchPointRelativeToField.toTranslation2d(), targetPointRelativeToField.toTranslation2d());
        Rotation2d altAng = firingSolution.getVectorRotation();

       return new BreakerTurretState(azAng, altAng, firingSolution.getMagnatude());
    }

    public BreakerTurretState calculateRobotRelative(Pose3d projectileLaunchPointRelativeToField, Translation3d targetPointRelativeToField) {
        BreakerTurretState calcState = calculateFieldRelative(projectileLaunchPointRelativeToField.getTranslation(), targetPointRelativeToField);
        return calcState.toRobotRelativeState(projectileLaunchPointRelativeToField.getRotation());
    }
}