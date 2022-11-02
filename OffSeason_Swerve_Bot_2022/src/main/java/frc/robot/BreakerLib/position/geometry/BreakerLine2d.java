// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.geometry;

import java.util.function.Function;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.BreakerLib.util.math.functions.BreakerGenericMathFunction;
import frc.robot.BreakerLib.util.math.functions.BreakerMathFunction;

/** Add your docs here. */
public class BreakerLine2d implements BreakerGenericMathFunction {
    private BreakerMathFunction mathFunc;
    public BreakerLine2d(Translation2d start, Translation2d end) {
        Function<Double, Double> func;
        if (start.getY() == end.getY()) {
            func = (Double x) -> (start.getY());
        } else if (start.getX() == end.getX()) {
            func = (Double x) -> (start.getX());
        } else {
            double m = calcM(start, end);
            double b = calcB(start, m);
            func = (Double x) -> ((m*x)+b);
        }
        mathFunc = new BreakerMathFunction(func);
    }

    private static double calcM(Translation2d p0, Translation2d p1) {
        return (p1.getY() - p0.getY()) / (p1.getX() - p0.getX());
    }

    private static double calcB(Translation2d p, double m) {
        return -(m*p.getX() - p.getY());
    }

    @Override
    public double getValueAtX(double xValue) {
        return mathFunc.getValueAtX(xValue);
    }

    @Override
    public BreakerGenericMathFunction add(BreakerGenericMathFunction funcToAdd) {
        return mathFunc.add(funcToAdd);
    }

    @Override
    public BreakerGenericMathFunction subtract(BreakerGenericMathFunction funcToSubtract) {
        return mathFunc.subtract(funcToSubtract);
    }

    @Override
    public BreakerGenericMathFunction multiply(BreakerGenericMathFunction multipul) {
        return mathFunc.multiply(multipul);
    }

    @Override
    public BreakerGenericMathFunction devide(BreakerGenericMathFunction dividend) {
        return mathFunc.devide(dividend);
    }
}
