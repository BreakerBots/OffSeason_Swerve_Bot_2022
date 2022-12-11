// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package BreakerLib.auto.trajectory.swerve.ltv;

import BreakerLib.position.odometry.BreakerGenericOdometer;
import BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import edu.wpi.first.math.controller.LTVUnicycleController;

/** Add your docs here. */
public class BreakerLTVUnicycleSwerveAutoPathFollowerConfig {
    
    private LTVUnicycleController unicycleController;
    private BreakerSwerveDrive drivetrain;
    private BreakerGenericOdometer odometer;
    public BreakerLTVUnicycleSwerveAutoPathFollowerConfig(BreakerSwerveDrive drivetrain, LTVUnicycleController unicycleController) {
        this(drivetrain, drivetrain, unicycleController);
    }

    public BreakerLTVUnicycleSwerveAutoPathFollowerConfig(BreakerSwerveDrive drivetrain, BreakerGenericOdometer odometer, LTVUnicycleController unicycleController) {
        this.unicycleController = unicycleController;
        this.drivetrain = drivetrain;
        this.odometer = odometer;
    }

    public LTVUnicycleController getUnicycleController() {
        return unicycleController;
    }

    public BreakerGenericOdometer getOdometer() {
        return odometer;
    }

    public BreakerSwerveDrive getDrivetrain() {
        return drivetrain;
    }
}
