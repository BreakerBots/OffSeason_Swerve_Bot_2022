// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.differential;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerCTREUtil;

/** A {@link BreakerDiffDrive} instance with TalonFX (Falcon 500) motors */
public class BreakerFalconDiffDrive extends BreakerDiffDrive {
    private WPI_TalonFX[] leftMotors;
    private WPI_TalonFX[] rightMotors;

     /** Creates a new Differential (tank drive) drivetrain instance.
     * 
     * @param leftMotors Left {@link WPI_TalonFX} motors.
     * @param invertL Invert left motor outputs & encoder readings.
     * @param rightMotors Right {@link WPI_TalonFX} motors.
     * @param invertR Invert right motor outputs & encoder readings.
     * @param gyro {@link BreakerGenericGyro} capable of reading yaw. 
     * @param driveConfig A {@link BreakerDiffDriveConfig} representing the configerable values of this drivetrain's kinimatics and control values
     */
    public BreakerFalconDiffDrive(WPI_TalonFX[] leftMotors, WPI_TalonFX[] rightMotors, boolean invertL, boolean invertR,
        BreakerGenericGyro imu, BreakerDiffDriveConfig driveConfig) {
        
        super(
            leftMotors,
            () -> ((Double)leftMotors[0].getSensorCollection().getIntegratedSensorPosition() / 2048.0),
            () -> ((Double)((leftMotors[0].getSensorCollection().getIntegratedSensorVelocity() * 600) / 2048.0)),
            invertL, 
            rightMotors,
            () -> ((Double)rightMotors[0].getSensorCollection().getIntegratedSensorPosition() / 2048.0),
            () -> ((Double)((rightMotors[0].getSensorCollection().getIntegratedSensorVelocity() * 600) / 2048.0)), 
            invertR,
            imu,
            driveConfig);
    }

    @Override
    public void runSelfTest() {
        faultStr = null;
        health = DeviceHealth.NOMINAL;

        StringBuilder work = new StringBuilder();
        for (WPI_TalonFX motorL : leftMotors) {
            Faults motorFaults = new Faults();
            motorL.getFaults(motorFaults);
            boolean motorDisconected = motorL.getFirmwareVersion() == -1;
            if (motorFaults.hasAnyFault() || motorDisconected) {
                health = DeviceHealth.FAULT;
                work.append(" MOTOR ID (" + motorL.getDeviceID() + ") FAULTS: ");
                if (motorFaults.hasAnyFault()) {
                    work.append(BreakerCTREUtil.getMotorFaultsAsString(motorFaults));
                }
                if (motorDisconected) {
                    work.append(" motor_disconected ");
                }
            }
        }
        for (WPI_TalonFX motorR : rightMotors) {
            Faults motorFaults = new Faults();
            motorR.getFaults(motorFaults);
            boolean motorDisconected = motorR.getFirmwareVersion() == -1;
            if (motorFaults.hasAnyFault() || motorDisconected) {
                health = DeviceHealth.FAULT;
                work.append(" MOTOR ID (" + motorR.getDeviceID() + ") FAULTS: ");
                if (motorFaults.hasAnyFault()) {
                    work.append(BreakerCTREUtil.getMotorFaultsAsString(motorFaults));
                }
                if (motorDisconected) {
                    work.append(" motor_disconected ");
                }
            }
        }
        faultStr = work.toString();
        
    }

    @Override
    public void resetDriveEncoders() {
        leftMotors[0].setSelectedSensorPosition(0);
        rightMotors[0].setSelectedSensorPosition(0);
    }

    @Override
    public void setDrivetrainBrakeMode(boolean isEnabled) {
        BreakerCTREUtil.setBrakeMode(isEnabled, leftMotors);
        BreakerCTREUtil.setBrakeMode(isEnabled, rightMotors);
    }

}
