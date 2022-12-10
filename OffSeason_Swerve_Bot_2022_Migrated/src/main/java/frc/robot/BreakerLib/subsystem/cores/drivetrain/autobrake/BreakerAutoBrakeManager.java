// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.autobrake;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain;
import frc.robot.BreakerLib.util.BreakerRoboRIO;

/**
 * Automatically handles brake mode switching for your drivetrain based on the
 * robot's current mode according to given config.
 */
public class BreakerAutoBrakeManager{

    private boolean brakeInAuto;
    private boolean brakeInTeleop;
    private boolean brakeInTest;
    private boolean brakeInDisabled;
    private boolean autoBrakeIsEnabled = true;
    private BreakerGenericDrivetrain baseDrivetrain;

    /** Creates an AutoBrake (automatic break mode) manager.
     * 
     * @param config Manager settings to use.
     */
    public BreakerAutoBrakeManager(BreakerAutoBrakeManagerConfig config) {
        brakeInAuto = config.getBreakInAuto();
        brakeInTeleop = config.getBreakInTeleop();
        brakeInTest = config.getBreakInTest();
        brakeInDisabled = config.getBreakInDisabled();
        baseDrivetrain = config.getBaseDrivetrain();

        CommandScheduler.getInstance().schedule(new RunCommand(() -> this.setAutomaticBreakMode()));
    }

    public void changeConfig(BreakerAutoBrakeManagerConfig config) {
        brakeInAuto = config.getBreakInAuto();
        brakeInTeleop = config.getBreakInTeleop();
        brakeInTest = config.getBreakInTest();
        brakeInDisabled = config.getBreakInDisabled();
        baseDrivetrain = config.getBaseDrivetrain();
    }

    public boolean isAutomaticBreakModeEnabled() {
        return autoBrakeIsEnabled;
    }

    public void setAutomaticBreakModeEnabled(Boolean isEnabled) {
        autoBrakeIsEnabled = isEnabled;
    }

    public void setAutomaticBreakMode() {
        switch (BreakerRoboRIO.getCurrentRobotMode()) {
            case DISABLED:
                baseDrivetrain.setDrivetrainBrakeMode(brakeInDisabled);
                break;
            case AUTONOMOUS:
                baseDrivetrain.setDrivetrainBrakeMode(brakeInAuto);
                break;
            case TELEOP:
                baseDrivetrain.setDrivetrainBrakeMode(brakeInTeleop);
                break;
            case TEST:
                baseDrivetrain.setDrivetrainBrakeMode(brakeInTest);
                break;
            case UNKNOWN:
            default:
                baseDrivetrain.setDrivetrainBrakeMode(false);
                break;

        }
    }

    public boolean getBrakeInAuto() {
        return brakeInAuto;
    }

    public boolean getBrakeInDisabled() {
        return brakeInDisabled;
    }

    public boolean getBrakeInTeleop() {
        return brakeInTeleop;
    }

    public boolean getBrakeInTest() {
        return brakeInTest;
    }
}
