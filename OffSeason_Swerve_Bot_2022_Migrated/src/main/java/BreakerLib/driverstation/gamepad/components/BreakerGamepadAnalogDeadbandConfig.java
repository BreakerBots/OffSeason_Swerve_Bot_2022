// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package BreakerLib.driverstation.gamepad.components;

/** Configuration for all axis deadbands for a given gamepad. */
public class BreakerGamepadAnalogDeadbandConfig {

    private double leftX = 0.0, leftY = 0.0, rightX = 0.0, rightY = 0.0, leftTriggerAxis = 0.0, rightTriggerAxis = 0.0;

    /**
     * Constructs deadband config for both sticks and triggers
     * 
     * @param leftX Left stick X-axis deadband.
     * @param leftY Left stick Y-axis deadband.
     * @param rightX Right stick X-axis deadband.
     * @param rightY Right stick Y-axis deadband.
     * @param leftTriggerAxis Left triggert deadband.
     * @param rightTriggerAxis Right trigger deadband.
     */
    public BreakerGamepadAnalogDeadbandConfig(double leftX, double leftY, double rightX, double rightY, double leftTriggerAxis, double rightTriggerAxis) {
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        this.rightY = rightY;
        this.leftTriggerAxis = leftTriggerAxis;
        this.rightTriggerAxis = rightTriggerAxis;
    }

    /**
     * Constructs deadband config for both analog sticks.
     * 
     * @param leftX Left stick X-axis deadband.
     * @param leftY Left stick Y-axis deadband.
     * @param rightX Right stick X-axis deadband.
     * @param rightY Right stick Y-axis deadband.
     */
    public BreakerGamepadAnalogDeadbandConfig(double leftX, double leftY, double rightX, double rightY) {
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        this.rightY = rightY;
        leftTriggerAxis = 0.0;
        rightTriggerAxis = 0.0;
    }

    /** Deadbands are all set to 0. */
    public BreakerGamepadAnalogDeadbandConfig() {}
    
    /** @return Left trigger deadband. */
    public double getLeftTriggerAxis() {
        return leftTriggerAxis;
    }

    /** @return Left stick X-axis deadband. */
    public double getLeftX() {
        return leftX;
    }
    
    /** @return Left stick Y-axis deadband. */
    public double getLeftY() {
        return leftY;
    }

    /** @return Right trigger deadband. */
    public double getRightTriggerAxis() {
        return rightTriggerAxis;
    }

    /** @return Right stick X-axis deadband. */
    public double getRightX() {
        return rightX;
    }

    /** @return Right stick Y-axis deadband. */
    public double getRightY() {
        return rightY;
    }
}
