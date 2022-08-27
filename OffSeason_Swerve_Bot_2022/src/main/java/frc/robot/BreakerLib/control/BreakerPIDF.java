// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.control;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** Add your docs here. */
public class BreakerPIDF {
    private PIDController pidController;
    private SimpleMotorFeedforward ffController;
    private double pidMinOutput, pidMaxOutput;
    public BreakerPIDF(PIDController pidController, SimpleMotorFeedforward ffController) {
        this.pidController = pidController;
        this.ffController = ffController;
        pidMinOutput = -Double.MAX_VALUE;
        pidMaxOutput = -Double.MIN_VALUE;
    }

    public BreakerPIDF(PIDController pidController, double pidMinOutput, double pidMaxOutput, SimpleMotorFeedforward ffController) {
        this.pidController = pidController;
        this.ffController = ffController;
        this.pidMaxOutput = pidMaxOutput;
        this.pidMinOutput = pidMinOutput;
    }

    public void setTolerence(double velocityTolerence, double accelerationTolerence) {
        pidController.setTolerance(velocityTolerence, accelerationTolerence);
    }

    public double calculate(double measurement, double setpoint) {
        return MathUtil.clamp(pidController.calculate(measurement, setpoint), pidMinOutput, pidMaxOutput)  + ffController.calculate(setpoint);
    }

    // public double calculatePrecentSpeed(double measurement, double setPoint, double voltage) {
    //     return pidController.calculate(measurement, setPoint) + (ffController.calculate(setPoint) / voltage);
    // }
    
    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    public double getVelocityError() {
        return pidController.getPositionError();
    }

    public SimpleMotorFeedforward getBaseFeedforwardController() {
        return ffController;
    }

    public PIDController getBasePidController() {
        return pidController;
    }
}
