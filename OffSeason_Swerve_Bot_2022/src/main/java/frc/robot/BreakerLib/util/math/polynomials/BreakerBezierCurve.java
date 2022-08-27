// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.polynomials;

import edu.wpi.first.math.geometry.Translation2d;

/** Beeeeeezier curves! */
public class BreakerBezierCurve implements BreakerGenericPolynomial {
    public double x0, y0;
    public double x1, y1;
    public double x2, y2;
    public double x3, y3;
  
    public BreakerBezierCurve(Translation2d controlPointOne, Translation2d controlPointTwo) {
        x0 = 0.0;
        y0 = 0.0;
        x1 = controlPointOne.getX();
        y1 = controlPointOne.getY();
        x2 = controlPointTwo.getX();
        y2 = controlPointTwo.getY();
        x3 = 1.0;
        y3 = 1.0;
    }

    public BreakerBezierCurve(Translation2d startPoint, Translation2d controlPointOne, Translation2d controlPointTwo, Translation2d endPoint) {
        x0 = startPoint.getX();
        y0 = startPoint.getY();
        x1 = controlPointOne.getX();
        y1 = controlPointOne.getY();
        x2 = controlPointTwo.getX();
        y2 = controlPointTwo.getY();
        x3 = endPoint.getX();
        y3 = endPoint.getY();
      }
  
    private double p(double x, double a, double b, double c) {
      return ((1 - x) * ((1 - x) * a + (x * b))) + (x * ((1 - x) * b + (x * c)));
    }

    @Override
    public double getValueAtX(double xValue) {
        xValue = (1 - xValue);
        xValue = 1 - (((1 - xValue) * p(xValue, x0, x1, x2)) + (xValue * p(xValue, x1, x2, x3)));
        xValue = (((1 - xValue) * p(xValue, y0, y1, y2)) + (xValue * p(xValue, y1, y2, y3)));
        return xValue;
    }

    @Override
    public double getSignRelativeValueAtX(double xValue) {
        boolean tn = xValue < 0;
        xValue = Math.abs(xValue);
        xValue = (1 - xValue);
        xValue = 1 - (((1 - xValue) * p(xValue, x0, x1, x2)) + (xValue * p(xValue, x1, x2, x3)));
        xValue = (((1 - xValue) * p(xValue, y0, y1, y2)) + (xValue * p(xValue, y1, y2, y3)));
      return xValue * (tn ? -1 : 1);
    }
}
