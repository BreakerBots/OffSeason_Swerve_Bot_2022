// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.robot;

import frc.robot.BreakerLib.auto.trajectory.management.BreakerAutoPath;
import frc.robot.BreakerLib.devices.cosmetic.music.BreakerFalconOrchestra;

/** Config class for the robot, used by {@link BreakerRobotManager}. */
public class BreakerRobotConfig {

    private double secondsBetweenSelfChecks;
    private boolean networkTablesLogging;
    private boolean autoRegisterDevices;
    private boolean useBrakeModeManager;
    private BreakerFalconOrchestra orchestra;
    private boolean usesOrchestra;
    private BreakerAutoPath[] autoPaths;
    private boolean usesPaths;
    private BreakerRobotStartConfig startConfig;

    /**
     * Default robot configuration. Pre-assigned settings are listed below:
     * <p>
     * - Seconds between self-checks = 5.
     * <p>
     * - Devices are automatically registered into SelfTest.
     * <p>
     * - Brake mode manager is enabled.
     * <p>
     * - NetworkTables aren't logged.
     * <p>
     * - Auto paths are not used.
     * <p>
     * - Falcon Orchestra is not used.
     * 
     * @param startConfig Startup configuration to use.
     */
    public BreakerRobotConfig(BreakerRobotStartConfig startConfig) {
        this.startConfig = startConfig;

        this.secondsBetweenSelfChecks = 5;
        this.networkTablesLogging = false;
        this.autoRegisterDevices = true;
        this.useBrakeModeManager = true;

        this.orchestra = new BreakerFalconOrchestra();
        usesOrchestra = false;

        this.autoPaths = new BreakerAutoPath[0];
        usesPaths = false;
    }

    /**
     * Allows for selection of auto paths on Shuffleboard GUI.
     * 
     * @param paths Autopaths to select from.
     */
    public void setAutoPaths(BreakerAutoPath... paths) {
        autoPaths = paths;
        usesPaths = true;
    }

    /**
     * Enables a message printed to the console on startup with robot info.
     * 
     * @param startConfig Robot info to use.
     */
    public void setStartConfig(BreakerRobotStartConfig startConfig) {
        this.startConfig = startConfig;
    }

    /**
     * Sets up use of Falcon Orchestra to play music.
     * 
     * @param orchestra Orchestra object to use.
     */
    public void setOrchestra(BreakerFalconOrchestra orchestra) {
        this.orchestra = orchestra;
        usesOrchestra = true;
    }

    /**
     * Sets whether or not to enable logging of NetworkTables.
     * 
     * @param enabled True for enable, false for disable.
     */
    public void setNetworkTablesLogging(boolean enabled) {
        networkTablesLogging = enabled;
    }

    /**
     * Sets whether or not to automatically register compatible devices into
     * BreakerLib's Self-Test functionality.
     * 
     * @param enabled True for enable, false for disable.
     */
    public void setAutoRegisterDevices(boolean enabled) {
        autoRegisterDevices = enabled;
    }

    /**
     * Sets whether or not to enable BrakeModeManager.
     * 
     * @param enabled True for enable, false for disable.
     */
    public void setBrakeModeManagerEnabled(boolean enabled) {
        useBrakeModeManager = enabled;
    }

    /**
     * Sets the number of seconds until another Self-Test is run.
     * 
     * @param seconds Seconds to wait.
     */
    public void setSecondsBetweenSelfTests(double seconds) {
        this.secondsBetweenSelfChecks = seconds;
    }

    public boolean getAutoRegisterDevices() {
        return autoRegisterDevices;
    }

    public BreakerAutoPath[] getAutoPaths() {
        return autoPaths;
    }

    public boolean getBrakeModeManagerEnabled() {
        return useBrakeModeManager;
    }

    public BreakerFalconOrchestra getOrchestra() {
        return orchestra;
    }

    public double getSecondsBetweenSelfChecks() {
        return secondsBetweenSelfChecks;
    }

    public boolean networkTablesLoggingEnabled() {
        return networkTablesLogging;
    }

    public boolean usesOrchestra() {
        return usesOrchestra;
    }

    public boolean usesPaths() {
        return usesPaths;
    }

    public BreakerRobotStartConfig getStartConfig() {
        return startConfig;
    }

}
