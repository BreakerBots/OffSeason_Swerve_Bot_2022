// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.polynomials;

/** Add your docs here. */
public class BreakerExponential implements BreakerGenericPolynomial {
    private double a, b, c, d;
    /** a(b^(x+c))+d */
    public BreakerExponential(double a, double b, double c, double d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }
    
    @Override
    public double getValueAtX(double xValue) {
        return (a * Math.pow(b, xValue + c)) + d;
    }

}
