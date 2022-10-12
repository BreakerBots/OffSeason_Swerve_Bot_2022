// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.interpolation;

import java.util.Arrays;

import frc.robot.BreakerLib.util.math.BreakerMath;

/**
 * Wraps around the double primitive type for use with BreakerLib's
 * interpolation classes
 */
public class BreakerInterpolatableDoubleArray implements BreakerInterpolable<BreakerInterpolatableDoubleArray> {
    private double[] values;

    public BreakerInterpolatableDoubleArray(double... values) {
        this.values = values;
    }

    public double[] getValue() {
        return values;
    }

    @Override
    public BreakerInterpolatableDoubleArray getSelf() {
        return this;
    }

    @Override
    public BreakerInterpolatableDoubleArray interpolate(double valToInterpolate, double highKey,
            BreakerInterpolatableDoubleArray highVal, double lowKey, BreakerInterpolatableDoubleArray lowVal) {
                double[] newArr = new double[values.length];
                for (int i = 0; i < values.length; i++) {
                    newArr[i] = BreakerMath.interpolateLinear(valToInterpolate, lowKey, highKey, lowVal.getValue()[i], highVal.getValue()[i]);
                }
        return new BreakerInterpolatableDoubleArray(newArr);
    }

    @Override
    public double[] getInterpolatableData() {
        return Arrays.copyOf(values, values.length);
    }

    @Override
    public BreakerInterpolatableDoubleArray fromInterpolatableData(double[] interpolatableData) {
        return new BreakerInterpolatableDoubleArray(interpolatableData);
    }

}
