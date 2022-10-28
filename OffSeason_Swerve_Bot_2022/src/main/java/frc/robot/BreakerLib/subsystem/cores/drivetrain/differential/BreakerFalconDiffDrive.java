// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.differential;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.test.vendorutil.BreakerCTREUtil;

/** Add your docs here. */
public class BreakerFalconDiffDrive extends BreakerDiffDrive {
    private WPI_TalonFX[] leftMotors;
    private WPI_TalonFX[] rightMotors;

    public BreakerFalconDiffDrive(WPI_TalonFX[] leftMotors, WPI_TalonFX[] rightMotors, boolean invertL, boolean invertR,
        BreakerGenericGyro imu, BreakerDiffDriveConfig driveConfig) {
        
        super(leftMotors,  () -> ((Double)leftMotors[0].getSelectedSensorPosition()), () -> ((Double)((leftMotors[0].getSelectedSensorVelocity() * 600) / driveConfig.getEncoderTicks())), invertL, 
            rightMotors, () -> ((Double)rightMotors[0].getSelectedSensorPosition()), () -> ((Double)((rightMotors[0].getSelectedSensorVelocity() * 600) / driveConfig.getEncoderTicks())), invertR,
                imu, driveConfig);
    }

    @Override
    public void runSelfTest() {
        faultStr = null;
        health = DeviceHealth.NOMINAL;

        StringBuilder work = new StringBuilder();
        for (WPI_TalonFX motorL : leftMotors) {
            Faults motorFaults = new Faults();
            motorL.getFaults(motorFaults);
            if (motorFaults.hasAnyFault()) {
                health = DeviceHealth.FAULT;
                work.append(" MOTOR ID (" + motorL.getDeviceID() + ") FAULTS: ");
                work.append(BreakerCTREUtil.getMotorFaultsAsString(motorFaults));
            }
        }
        for (WPI_TalonFX motorR : rightMotors) {
            Faults motorFaults = new Faults();
            motorR.getFaults(motorFaults);
            if (motorFaults.hasAnyFault()) {
                health = DeviceHealth.FAULT;
                work.append(" MOTOR ID (" + motorR.getDeviceID() + ") FAULTS: ");
                work.append(BreakerCTREUtil.getMotorFaultsAsString(motorFaults));
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
