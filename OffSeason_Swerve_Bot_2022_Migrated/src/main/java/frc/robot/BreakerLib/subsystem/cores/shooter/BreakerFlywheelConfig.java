// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.shooter;

import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;

/** Add your docs here. */
public class BreakerFlywheelConfig {
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double velocityTolerence;
    private double acclerationTolerence;
    private double flywheelGearRatio;
    private BreakerArbitraryFeedforwardProvider arbFFProvider;

    public BreakerFlywheelConfig(double kP, double kI, double kD, double kF, double velocityTolerence, double accelerationTolerence, double flywheelGearRatio, BreakerArbitraryFeedforwardProvider arbFFProvider) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.velocityTolerence= velocityTolerence;
        this.acclerationTolerence = accelerationTolerence;
        this.flywheelGearRatio = flywheelGearRatio;
        this.arbFFProvider = arbFFProvider;
    }

    
    /** 
     * @return double
     */
    public double getkD() {
        return kD;
    }
    
    /** 
     * @return double
     */
    public double getkF() {
        return kF;
    }

    
    /** 
     * @return double
     */
    public double getkI() {
        return kI;
    }

    
    /** 
     * @return double
     */
    public double getkP() {
        return kP;
    }

    
    /** 
     * @return double
     */
    public double getAcclerationTolerence() {
        return acclerationTolerence;
    }

    
    /** 
     * @return double
     */
    public double getVelocityTolerence() {
        return velocityTolerence;
    }

    
    /** 
     * @return double
     */
    public double getFlywheelGearRatio() {
        return flywheelGearRatio;
    }

    
    /** 
     * @return BreakerArbitraryFeedforwardProvider
     */
    public BreakerArbitraryFeedforwardProvider getArbFFProvider() {
        return arbFFProvider;
    }
}
