// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerAutoManager;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.test.selftest.SelfTest;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.brakemode.BreakerBrakeModeManager;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.brakemode.BreakerBrakeModeManagerConfig;

/**
 * Robot manager that configures SelfTest functionality, automatic brake mode,
 * and auto paths.
 */
public class BreakerRobotManager {
    private static SelfTest test;
    private static BreakerAutoManager autoManager;
    private static BreakerBrakeModeManager brakeModeManager;
    private static BreakerGenericDrivetrain baseDrivetrain;

    private BreakerRobotManager() {
    }

    /** Setup for the BreakerRobotManager.
     * 
     * @param baseDrivetrain Base drivetrain.
     * @param robotConfig Robot configuration.
     */
    public static void setup(BreakerGenericDrivetrain baseDrivetrain, BreakerRobotConfig robotConfig) {
        if (robotConfig.usesOrchestra()) {
            BreakerLog.startLog(robotConfig.networkTablesLoggingEnabled(), robotConfig.getOrchestra());
            test = new SelfTest(robotConfig.getSecondsBetweenSelfChecks(),
                    robotConfig.getOrchestra(), robotConfig.getAutoRegisterDevices());
        } else {
            BreakerLog.startLog(robotConfig.networkTablesLoggingEnabled());
            test = new SelfTest(robotConfig.getSecondsBetweenSelfChecks(),
                    robotConfig.getAutoRegisterDevices());
        }
        BreakerRobotManager.baseDrivetrain = baseDrivetrain;
        BreakerRobotManager.autoManager = robotConfig.usesPaths() ? new BreakerAutoManager(robotConfig.getAutoPaths())
                : new BreakerAutoManager();
        BreakerRobotManager.brakeModeManager = new BreakerBrakeModeManager(
                new BreakerBrakeModeManagerConfig(baseDrivetrain));
        BreakerLog.logRobotStarted(robotConfig.getStartConfig());
    }

    /** @return Brake mode manager object. */
    public static BreakerBrakeModeManager getBrakeModeManager() {
        return brakeModeManager;
    }

    /** @return SelfTest object. */
    public static SelfTest getSelfTest() {
        return test;
    }

    /** @return Auto manager object. */
    public static BreakerAutoManager getAutoManager() {
        return autoManager;
    }

    /** @return Autopath selected through auto manager. */
    public static Command getSelectedAutoPath() {
        return autoManager.getSelectedAutoPath();
    }

    /** Enable or disable brake mode. */
    public static void setDrivetrainBrakeMode(boolean isEnabled) {
        baseDrivetrain.setDrivetrainBrakeMode(isEnabled);
    }
}
