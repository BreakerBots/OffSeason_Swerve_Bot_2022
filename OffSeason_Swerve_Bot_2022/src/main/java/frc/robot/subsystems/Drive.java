// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerMK4iFalconSwerveModule;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;

public class Drive extends SubsystemBase {

    private BreakerSwerveDriveConfig config;
    private BreakerSwerveDrive drivetrain;

    private WPI_TalonFX driveFL, turnFL;
    private WPI_TalonFX driveFR, turnFR;
    private WPI_TalonFX driveBL, turnBL;
    private WPI_TalonFX driveBR, turnBR;

    private WPI_CANCoder encoderFL, encoderFR, encoderBL, encoderBR;

    private Translation2d transFL, transFR, transBL, transBR;

    private BreakerMK4iFalconSwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;

    public Drive(BreakerPigeon2 pigeon2) {

        driveFL = new WPI_TalonFX(FL_WHEEL_ID);
        turnFL = new WPI_TalonFX(FL_ROTATION_ID);
        encoderFL = new WPI_CANCoder(FL_ENCODER_ID);
        transFL = FL_TRANSLATION;

        driveFR = new WPI_TalonFX(FR_WHEEL_ID);
        turnFR = new WPI_TalonFX(FR_ROTATION_ID);
        encoderFR = new WPI_CANCoder(FR_ENCODER_ID);
        transFR = FR_TRANSLATION;

        driveBL = new WPI_TalonFX(BL_WHEEL_ID);
        turnBL = new WPI_TalonFX(BL_ROTATION_ID);
        encoderBL = new WPI_CANCoder(BL_ENCODER_ID);
        transBL = BL_TRANSLATION;

        driveBR = new WPI_TalonFX(BR_WHEEL_ID);
        turnBR = new WPI_TalonFX(BR_ROTATION_ID);
        encoderBR = new WPI_CANCoder(BR_ENCODER_ID);
        transBR = BR_TRANSLATION;

        config = new BreakerSwerveDriveConfig(4.1148, 4.1148, 16.1148, 1.25, 0.0, 0.05, 0.35, 0.0, 0.0, 0.0, 8.14, 4.0, 0.001, 4.1148,
                new BreakerArbitraryFeedforwardProvider(2.75, 0.2), transFL, transFR, transBL, transBR);
        config.setSlowModeMultipliers(0.5, 0.5);

        frontLeftModule = new BreakerMK4iFalconSwerveModule(driveFL, turnFL, encoderFL, config, 121, true, true);
        frontLeftModule.setDeviceName(" FL_Module ");

        frontRightModule = new BreakerMK4iFalconSwerveModule(driveFR, turnFR, encoderFR, config, -61, false, true);
        frontRightModule.setDeviceName(" FR_Module ");

        backLeftModule = new BreakerMK4iFalconSwerveModule(driveBL, turnBL, encoderBL, config, 30.0, true, true);
        backLeftModule.setDeviceName(" BL_Module ");

        backRightModule = new BreakerMK4iFalconSwerveModule(driveBR, turnBR, encoderBR, config, -176.0, false, true);
        backRightModule.setDeviceName(" BR_Module ");

        drivetrain = new BreakerSwerveDrive(config, pigeon2, frontLeftModule, frontRightModule, backLeftModule,
                backRightModule);
    }

    public BreakerSwerveDrive getBaseDrivetrain() {
        return drivetrain;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Drive Pose", drivetrain.getOdometryPoseMeters().toString());
    }
}
