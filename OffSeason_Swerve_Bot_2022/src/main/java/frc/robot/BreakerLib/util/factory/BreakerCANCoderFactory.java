// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.factory;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import frc.robot.BreakerLib.util.BreakerCTREUtil;

/** Factory for producing CANcoders. */
public class BreakerCANCoderFactory {
    public static WPI_CANCoder createCANCoder(int deviceID, AbsoluteSensorRange absoluteSensorRange, double absoluteOffsetDegress, boolean encoderDirection) {
        WPI_CANCoder encoder = new WPI_CANCoder(deviceID);
        BreakerCTREUtil.checkError(encoder.configAbsoluteSensorRange(absoluteSensorRange), " CANCoder " + deviceID + " factory ABS sensor range config fail ");
        BreakerCTREUtil.checkError(encoder.configMagnetOffset(absoluteOffsetDegress), " CANCoder " + deviceID + " factory mag offest angle config fail ");
        BreakerCTREUtil.checkError(encoder.configSensorDirection(encoderDirection), " CANCoder " + deviceID + " factory sensor direction config fail ");
        return encoder;
    }
}
