// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.mathfunctions;

import java.util.function.Function;

/** Add your docs here. */
public class BreakerCubic extends BreakerMathFunction {

    public BreakerCubic(double a, double b, double c, double d) {
        super((Double x) -> (a*(x*x*x) + b*(x*x) + c*x + d));
        //TODO Auto-generated constructor stub
    }
    
}
