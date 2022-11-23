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
    private  BreakerInterpolatingTreeMap<Double, BreakerInterpolablePair<BreakerVector2, BreakerInterpolableDouble>> fireingTable;
    private BreakerProjectile projectile;
    /** @param projectile
     *  @param fireingTable fireing distance compared to BreakerVector2(angle and fireing vel m/s) and flywheel RPM
      */
    public BreakerTurretController(BreakerProjectile projectile, BreakerInterpolatingTreeMap<Double, BreakerInterpolablePair<BreakerVector2, BreakerInterpolableDouble>> fireingTable) {

    }

    public BreakerTurretState calculateFieldRelative(Translation3d projectileLaunchPointRelativeToField, Translation3d targetPointRelativeToField) {
        double distance = projectileLaunchPointRelativeToField.toTranslation2d().getDistance(targetPointRelativeToField.toTranslation2d());
        BreakerInterpolablePair<BreakerVector2, BreakerInterpolableDouble> fireingPair = fireingTable.getInterpolatedValue(distance);

        Rotation2d azAng = BreakerMath.getPointAngleRelativeToOtherPoint(projectileLaunchPointRelativeToField.toTranslation2d(), targetPointRelativeToField.toTranslation2d());
        Rotation2d altAng = fireingPair.getFirst().getVectorRotation();

       return new BreakerTurretState(azAng, altAng, fireingPair.getSecond().getValue());
    }

    public BreakerTurretState calculateFieldRelative() {
        return null;
    }
}