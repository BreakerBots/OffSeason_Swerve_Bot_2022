// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.swerve.rotation;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.DoubleAccumulator;
import java.util.function.Function;
import java.util.function.Supplier;

import javax.print.Doc;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolableDouble;
import frc.robot.BreakerLib.util.math.interpolation.interpolateingmaps.BreakerInterpolatingTreeMap;

/** Add your docs here. */
public class BreakerSwerveRotationSupplier implements BreakerGenericSwerveRotationSupplier {
    private BreakerRotationPoint[] rotationPoints;
    private Function<Double, Rotation2d> externalFunction;
    private BreakerInterpolatingTreeMap<Double, BreakerInterpolableDouble> interMap;
    private boolean  usesFunc;
    public BreakerSwerveRotationSupplier(BreakerRotationPoint... rotationPoints) {
        this.rotationPoints = rotationPoints;
        usesFunc = false;
        for (BreakerRotationPoint point: rotationPoints) {
            interMap.put(point.getTimeOfRotation(), new BreakerInterpolableDouble(point.getRotation().getRadians()));
        }
    }

    public BreakerSwerveRotationSupplier(List<BreakerRotationPoint> rotationPoints) {
        this.rotationPoints = rotationPoints.toArray(new BreakerRotationPoint[rotationPoints.size()]);
        usesFunc = false;
        for (BreakerRotationPoint point: rotationPoints) {
            interMap.put(point.getTimeOfRotation(), new BreakerInterpolableDouble(point.getRotation().getRadians()));
        }
    }

    public BreakerSwerveRotationSupplier(Supplier<Rotation2d> externalSupplier) {
        this.externalFunction = (Double time) -> (externalSupplier.get());
        usesFunc = true;
    }

    /** function must take an input of time in seconds */
    public BreakerSwerveRotationSupplier(Function<Double, Rotation2d> externalFunction) {
        this.externalFunction = externalFunction;
        usesFunc = true;
    }

    @Override
    public Rotation2d getRotation(double currentTime) {
        if (usesFunc) {
            return externalFunction.apply(currentTime);
        }
        return new Rotation2d(interMap.getInterpolatedValue(currentTime).getValue());
    }

}
