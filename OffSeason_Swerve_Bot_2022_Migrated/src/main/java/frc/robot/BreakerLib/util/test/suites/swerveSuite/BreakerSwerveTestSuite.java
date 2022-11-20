// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.test.suites.swerveSuite;

import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerGenericSwerveModule;
import frc.robot.BreakerLib.util.test.suites.BreakerTestSuiteDataLogType;

/** Add your docs here. */
public class BreakerSwerveTestSuite {
    private BreakerSwerveDrive drivetrain;
    private BreakerTestSuiteDataLogType logType;
    private BreakerGenericSwerveModule[] modules;
    public BreakerSwerveTestSuite(BreakerSwerveDrive drivetrain, BreakerGenericSwerveModule... modules) {
        this.drivetrain = drivetrain;
        this.modules = modules;
    }

    public void setLogType(BreakerTestSuiteDataLogType newLogType) {
        logType = newLogType;
    }

    public BreakerSwerveModuleVelocityLimitTest timedSpinUpTest(double testTimeSeconds, double targetMaxSpeed) {
        BreakerSwerveModuleVelocityLimitTest test = new BreakerSwerveModuleVelocityLimitTest(drivetrain, modules, targetMaxSpeed, testTimeSeconds, logType);
        test.schedule();
        return test;
    }




}