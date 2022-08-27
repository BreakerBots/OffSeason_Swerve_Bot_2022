// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.polynomials;

/** Add your docs here. */
public interface BreakerGenericPolynomial {
    public abstract double getValueAtX(double xValue);
    public abstract double getSignRelativeValueAtX(double xValue);
}
