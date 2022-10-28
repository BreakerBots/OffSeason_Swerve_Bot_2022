// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.functions;

/** A interface for all BreakerLib classes that represent pure mathmatical funcions (or equations) such as Bezier Curves or Quadratic functions*/
public interface BreakerGenericMathFunction {
    public abstract double getValueAtX(double xValue);
    public default double getSignRelativeValueAtX(double xValue) {
        return getValueAtX(Math.abs(xValue)) * (xValue < 0 ? -1 : 1);
    }

    public abstract BreakerGenericMathFunction add(BreakerGenericMathFunction funcToAdd);

    public abstract BreakerGenericMathFunction subtract(BreakerGenericMathFunction funcToSubtract);

    public abstract BreakerGenericMathFunction multiply(BreakerGenericMathFunction multipul);

    public abstract BreakerGenericMathFunction devide(BreakerGenericMathFunction dividend);
}
