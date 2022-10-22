// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.mathfunctions;

/** Add your docs here. */
public class BreakerPolynomial implements BreakerGenericMathFunction {
    public BreakerMonomial[] monomials;
    public BreakerPolynomial(BreakerMonomial...monomials) {
        this.monomials = monomials;
    }

    @Override
    public double getValueAtX(double xValue) {
        double total = 0;
        for (BreakerMonomial mon: monomials) {
            total += mon.getValueAtX(xValue);
        }
        return total;
    }

    @Override
    public BreakerGenericMathFunction add(BreakerGenericMathFunction funcToAdd) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public BreakerGenericMathFunction subtract(BreakerGenericMathFunction funcToSubtract) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public BreakerGenericMathFunction multiply(BreakerGenericMathFunction multipul) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public BreakerGenericMathFunction devide(BreakerGenericMathFunction dividend) {
        // TODO Auto-generated method stub
        return null;
    }

}
