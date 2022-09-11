// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.sensors.BreakerPigeon2;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerMK4iSwerveModule;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;

/** Add your docs here. */
public class Drive extends SubsystemBase {

    private BreakerSwerveDriveConfig config;
    private BreakerSwerveDrive drivetrain;

    private WPI_TalonFX driveFL, turnFL;
    private WPI_TalonFX driveFR, turnFR;
    private WPI_TalonFX driveBL, turnBL;
    private WPI_TalonFX driveBR, turnBR;

    private WPI_CANCoder encoderFL, encoderFR, encoderBL, encoderBR;

    private Translation2d transFL, transFR, transBL, transBR;

    private BreakerMK4iSwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;

    private BreakerPigeon2 pigeon2;

    public Drive(BreakerPigeon2 pigeon2) {
        this.pigeon2 = pigeon2;

        driveFL = new WPI_TalonFX(FL_WHEEL_ID);
        turnFL = new WPI_TalonFX(FL_ROTATION_ID);
        encoderFL = BreakerCANCoderFactory.createCANCoder(FL_ENCODER_ID, AbsoluteSensorRange.Signed_PlusMinus180, 0.0,
                true);
        transFL = FL_TRANSLATION;

        driveFR = new WPI_TalonFX(FR_WHEEL_ID);
        turnFR = new WPI_TalonFX(FR_ROTATION_ID);
        encoderFR = BreakerCANCoderFactory.createCANCoder(FR_ENCODER_ID, AbsoluteSensorRange.Signed_PlusMinus180, 0.0,
                false);
        transFR = FR_TRANSLATION; // Yousif be like

        driveBL = new WPI_TalonFX(BL_WHEEL_ID);
        turnBL = new WPI_TalonFX(BL_ROTATION_ID);
        encoderBL = BreakerCANCoderFactory.createCANCoder(BL_ENCODER_ID, AbsoluteSensorRange.Signed_PlusMinus180, 0.0,
                false);
        transBL = BL_TRANSLATION;

        driveBR = new WPI_TalonFX(BR_WHEEL_ID);
        turnBR = new WPI_TalonFX(BR_ROTATION_ID);
        encoderBR = BreakerCANCoderFactory.createCANCoder(BR_ENCODER_ID, AbsoluteSensorRange.Signed_PlusMinus180, 0.0,
                true);
        transBR = BR_TRANSLATION;

        config = new BreakerSwerveDriveConfig(4.1148, 4.1148, 4.1148, 0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 8.14, 4.0,
                new BreakerArbitraryFeedforwardProvider(1.0, 0.0), transFL, transFR, transBL, transBR);
        config.setSlowModeMultipliers(0.5, 0.5);

        frontLeftModule = new BreakerMK4iSwerveModule(driveFL, turnFL, encoderFL, config);
        frontLeftModule.setDeviceName(" Front_Left_Module ");
        frontRightModule = new BreakerMK4iSwerveModule(driveFR, turnFR, encoderFR, config);
        frontRightModule.setDeviceName(" Front_Right_Module ");
        backLeftModule = new BreakerMK4iSwerveModule(driveBL, turnBL, encoderBL, config);
        backLeftModule.setDeviceName(" Back_Left_Module ");
        backRightModule = new BreakerMK4iSwerveModule(driveBR, turnBR, encoderBR, config);
        backRightModule.setDeviceName(" Back_Right_Module ");

        drivetrain = new BreakerSwerveDrive(config, pigeon2, frontLeftModule, frontRightModule, backLeftModule,
                backRightModule);
    }

    public BreakerSwerveDrive getBaseDrivetrain() {
        return drivetrain;
    }

    @Override
    public void periodic() {
        drivetrain.updateOdometry();
    }
}
