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
    private Supplier<Rotation2d> externalSupplier;
    private Function<Double, Rotation2d> externalFunction;
    private BreakerInterpolatingTreeMap<Double, BreakerInterpolableDouble> interMap;
    private boolean usesSupplier, usesFunc;
    private double currentTime = 0.0;
    public BreakerSwerveRotationSupplier(BreakerRotationPoint... rotationPoints) {
        this.rotationPoints = rotationPoints;
        usesSupplier = false;
        usesFunc = false;
        for (BreakerRotationPoint point: rotationPoints) {
            interMap.put(point.getTimeOfRotation(), new BreakerInterpolableDouble(point.getRotation().getRadians()));
        }
    }

    public BreakerSwerveRotationSupplier(List<BreakerRotationPoint> rotationPoints) {
        this.rotationPoints = rotationPoints.toArray(new BreakerRotationPoint[rotationPoints.size()]);
        usesSupplier = false;
        usesFunc = false;
        for (BreakerRotationPoint point: rotationPoints) {
            interMap.put(point.getTimeOfRotation(), new BreakerInterpolableDouble(point.getRotation().getRadians()));
        }
    }

    public BreakerSwerveRotationSupplier(Supplier<Rotation2d> externalSupplier) {
        this.externalSupplier = externalSupplier;
        usesSupplier = true;
        usesFunc = false;
    }

    /** function must take an input of time in seconds */
    public BreakerSwerveRotationSupplier(Function<Double, Rotation2d> externalFunction) {
        this.externalFunction = externalFunction;
        usesSupplier = false;
        usesFunc = true;
    }

    @Override
    public void setCurrentTime(double currentTime) {
        this.currentTime = currentTime;
    }

    @Override
    public BreakerRotationPoint[] getRotationPoints() {
        return rotationPoints;
    }

    @Override
    public Rotation2d getRotation() {
        if (usesSupplier) {
            return externalSupplier.get();
        } else if (usesFunc) {
            return externalFunction.apply(currentTime);
        }
        return new Rotation2d(interMap.getInterpolatedValue(currentTime).getValue());
    }

    /** this will be assumed to be first */
    public BreakerSwerveRotationSupplier concatenate(BreakerSwerveRotationSupplier outher) {
        List<BreakerRotationPoint> rotationPointList = new ArrayList<>();
        for (BreakerRotationPoint point: rotationPoints) {
            rotationPointList.add(point);
        }
        for (BreakerRotationPoint point: outher.getRotationPoints()) {
           rotationPointList.add(new BreakerRotationPoint(point.getRotation(), point.getTimeOfRotation() + rotationPoints[rotationPoints.length - 1].getTimeOfRotation()));
        }
        return new BreakerSwerveRotationSupplier(rotationPointList);
    }
}
