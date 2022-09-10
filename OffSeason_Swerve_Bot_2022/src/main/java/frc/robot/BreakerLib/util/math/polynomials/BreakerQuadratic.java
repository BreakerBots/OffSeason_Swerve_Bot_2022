// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.polynomials;

import java.util.ArrayList;
import java.util.List;

/** Add your docs here. */
public class BreakerQuadratic implements BreakerGenericPolynomial {
    BreakerPolynomial quad;
    public BreakerQuadratic(double a, double b, double c) {
        quad = new BreakerPolynomial(new BreakerMonomial(a, 2.0), new BreakerMonomial(b, 1.0), new BreakerMonomial(c));
    }

    public BreakerQuadratic(BreakerPolynomial polynomial) {
        List<BreakerMonomial> monomialsToAdd = new ArrayList<>();
        for (BreakerMonomial mon: polynomial.monomials) {
            if(mon.getDegree() <= 2.0) {
                monomialsToAdd.add(mon);
            }
        }
        quad = new BreakerPolynomial(monomialsToAdd.toArray(new BreakerMonomial[monomialsToAdd.size()]));
    }

    public BreakerQuadratic() {
        quad = new BreakerPolynomial(new BreakerMonomial(1.0, 2.0));
    }

    @Override
    public double getValueAtX(double xValue) {
        return quad.getValueAtX(xValue);
    }

    @Override
    public double getSignRelativeValueAtX(double xValue) {
        return quad.getSignRelativeValueAtX(xValue);
    }

    public BreakerPolynomial getBasePolynomial() {
        return quad;
    }
}
