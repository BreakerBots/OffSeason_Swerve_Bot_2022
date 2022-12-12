// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerGenericSwerveModule;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerMK4iFalconSwerveModule;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;
import static frc.robot.Constants.*;

/** Add your docs here. */
public class Drive extends BreakerSwerveDrive {
    
    private static WPI_TalonFX driveFL = new WPI_TalonFX(FL_WHEEL_ID);
    private static WPI_TalonFX turnFL = new WPI_TalonFX(FL_ROTATION_ID);
    private static WPI_CANCoder encoderFL = new WPI_CANCoder(FL_ENCODER_ID);

    private static WPI_TalonFX driveFR = new WPI_TalonFX(FR_WHEEL_ID);
    private static WPI_TalonFX turnFR = new WPI_TalonFX(FR_ROTATION_ID);
    private static WPI_CANCoder encoderFR = new WPI_CANCoder(FR_ENCODER_ID);

    private static WPI_TalonFX driveBL = new WPI_TalonFX(BL_WHEEL_ID);
    private static WPI_TalonFX turnBL = new WPI_TalonFX(BL_ROTATION_ID);
    private static WPI_CANCoder encoderBL = new WPI_CANCoder(BL_ENCODER_ID);

    private static WPI_TalonFX driveBR = new WPI_TalonFX(BR_WHEEL_ID);
    private static WPI_TalonFX turnBR = new WPI_TalonFX(BR_ROTATION_ID);
    private static WPI_CANCoder encoderBR = new WPI_CANCoder(BR_ENCODER_ID);

    private static BreakerSwerveDriveConfig config = new BreakerSwerveDriveConfig(
        4.1148, 4.1148, 16.1148,
        1.25, 0.0, 0.05,
        0.35, 0.0, 0.0, 0.0,
        8.14, 4.0, 0.001, 4.1148,
        new BreakerArbitraryFeedforwardProvider(2.75, 0.2),
        FL_TRANSLATION, FR_TRANSLATION, BL_TRANSLATION, BR_TRANSLATION)
        .setSlowModeMultipliers(0.5, 0.5);

    private static BreakerMK4iFalconSwerveModule frontLeftModule = new BreakerMK4iFalconSwerveModule(driveFL, turnFL, encoderFL, config, 121, true, true);
    private static BreakerMK4iFalconSwerveModule frontRightModule = new BreakerMK4iFalconSwerveModule(driveFR, turnFR, encoderFR, config, -61, false, true);
    private static BreakerMK4iFalconSwerveModule backLeftModule = new BreakerMK4iFalconSwerveModule(driveBL, turnBL, encoderBL, config, 30.0, true, true);
    private static BreakerMK4iFalconSwerveModule backRightModule = new BreakerMK4iFalconSwerveModule(driveBR, turnBR, encoderBR, config, -176.0, false, true);
    
    public Drive(BreakerPigeon2 imu) {
        super(config, imu, frontLeftModule, frontRightModule, backLeftModule, backRightModule);
        frontLeftModule.setDeviceName(" FL_Module ");
        frontRightModule.setDeviceName(" FR_Module ");
        backLeftModule.setDeviceName(" BL_Module ");
        backRightModule.setDeviceName(" BR_Module ");
    }

}
