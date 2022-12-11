// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package BreakerLib.auto.trajectory.diff;

import BreakerLib.position.odometry.BreakerGenericOdometer;
import BreakerLib.subsystem.cores.drivetrain.differential.BreakerDiffDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** Add your docs here. */
public class BreakerRamseteConfig {
    private BreakerDiffDrive drivetrain;
    private BreakerGenericOdometer odometer;
    private RamseteController ramseteController;
    private PIDController leftDriveController, rightDriveController;
    private SimpleMotorFeedforward ffDriveController;
    public BreakerRamseteConfig(BreakerDiffDrive drivetrain, PIDController leftDriveController, PIDController rightDriveController, SimpleMotorFeedforward ffDriveController, RamseteController ramseteController) {
        this.drivetrain = drivetrain;
        this.ramseteController = ramseteController;
        this.leftDriveController = leftDriveController;
        this.rightDriveController = rightDriveController;
        this.ffDriveController = ffDriveController;
        odometer = drivetrain;
    }

    public BreakerRamseteConfig(BreakerDiffDrive drivetrain, BreakerGenericOdometer odometer, PIDController leftDriveController, PIDController rightDriveController, SimpleMotorFeedforward ffDriveController, RamseteController ramseteController) {
        this.drivetrain = drivetrain;
        this.ramseteController  = ramseteController;
        this.leftDriveController = leftDriveController;
        this.rightDriveController = rightDriveController;
        this.ffDriveController = ffDriveController;
        this.odometer = odometer;
    }


    public BreakerDiffDrive getDrivetrain() {
        return drivetrain;
    }

    public BreakerGenericOdometer getOdometer() {
        return odometer;
    }

    public RamseteController getRamseteController() {
        return ramseteController;
    }

    public PIDController getLeftDriveController() {
        return leftDriveController;
    }

    public PIDController getRightDriveController() {
        return rightDriveController;
    }

    public SimpleMotorFeedforward getFeedforward() {
        return ffDriveController;
    }
}
