// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

/** Breaker interface for DIO (digital input-output) devices. */
public class BreakerDIO {

    /** Current state of given DIO device. */
    public enum DIOState {
        DIGITAL_IN,
        DIGITAL_OUT,
        PWM_OUT
    }

    private DigitalInput input;
    private DigitalOutput output;

    private DIOState state = DIOState.DIGITAL_IN;
    private int channel;

    /**
     * Constructor for DIO object.
     * 
     * @param channel DIO channel on the RoboRIO/MXP.
     */
    public BreakerDIO(int channel) {
        input = new DigitalInput(channel);
        output = new DigitalOutput(channel);
        this.channel = channel;
    }

    
    /** 
     * @return boolean
     */
    public boolean getInput() {
        output.disablePWM();
        state = DIOState.DIGITAL_IN;
        return input.get();
    }

    
    /** 
     * @return boolean
     */
    public boolean getOutput() {
        state = DIOState.DIGITAL_OUT;
        return output.get();
    }

    
    /** 
     * @param value
     */
    public void setOutput(boolean value) {
        output.disablePWM();
        output.set(value);
    }

    
    /** 
     * @param dutyCycle
     */
    public void setOutputPWM(double dutyCycle) {
        state = DIOState.PWM_OUT;
        output.enablePWM(dutyCycle);
    }

    
    /** 
     * @return DIOState
     */
    public DIOState getState() {
        return state;
    }

    
    /** 
     * @return DigitalInput
     */
    public DigitalInput getBaseInput() {
        return input;
    }

    
    /** 
     * @return DigitalOutput
     */
    public DigitalOutput getBaseOutput() {
        return output;
    }

    
    /** 
     * @return int
     */
    public int getChannel() {
        return channel;
    }

}