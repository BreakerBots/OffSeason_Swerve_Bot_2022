// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.BreakerLib.util.math.interpolation.interpolateingmaps.BreakerGenericInterpolatingMap;

/** A class that acts as a provider for arbitrary feedforward demand values used with the Talon motor controller's integrated PID. */
public class BreakerArbitraryFeedforwardProvider {

    private BreakerGenericInterpolatingMap<Double, Double> ffMap;
    private double feedforwardCoeficent, staticFrictionCoeficent; 
    private Function<Double, Double> ffFunc;
    private SimpleMotorFeedforward ffClass;

    private FeedForwardType ffType;

    private enum FeedForwardType{
        MAP_SUP,
        COEFS,
        FUNC,
        FF_CLASS
    }

    public BreakerArbitraryFeedforwardProvider(BreakerGenericInterpolatingMap<Double, Double> speedToFeedforwardValMap) {
        ffMap = speedToFeedforwardValMap;
        ffType = FeedForwardType.MAP_SUP;
    }

    public BreakerArbitraryFeedforwardProvider(double feedforwardCoeficent, double staticFrictionCoeficent) {
        this.feedforwardCoeficent = feedforwardCoeficent;
        this.staticFrictionCoeficent = staticFrictionCoeficent;
        ffType = FeedForwardType.COEFS;
    }

    public BreakerArbitraryFeedforwardProvider(Function<Double, Double> ffFunc) {
        this.ffFunc = ffFunc;
        ffType = FeedForwardType.FUNC;
    }

    public BreakerArbitraryFeedforwardProvider(DoubleSupplier ffSupplier) {
        ffFunc = (Double x) -> (ffSupplier.getAsDouble());
        ffType = FeedForwardType.FUNC;
    }

    public BreakerArbitraryFeedforwardProvider(SimpleMotorFeedforward ffClass) {
        this.ffClass = ffClass;
        ffType = FeedForwardType.FF_CLASS;
    }

    public double getArbitraryFeedforwardValue(double curSpeed) {
        double feedForward = 0.0;
        switch (ffType) {
            case COEFS:
                feedForward = (feedforwardCoeficent * curSpeed + staticFrictionCoeficent) / RobotController.getBatteryVoltage();
                break;
            case MAP_SUP:
                feedForward = ffMap.getInterpolatedValue(curSpeed);
                break;
            case FUNC:
                feedForward = ffFunc.apply(curSpeed);
                break;
            case FF_CLASS:
                feedForward = ffClass.calculate(curSpeed) / RobotController.getBatteryVoltage();
        }
        return feedForward;
    }
}
