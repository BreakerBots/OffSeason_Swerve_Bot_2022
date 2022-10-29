// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.robotmanager;

import frc.robot.BreakerLib.auto.trajectory.management.BreakerAutoPath;
import frc.robot.BreakerLib.devices.cosmetic.music.BreakerFalconOrchestra;

/** Configuration for the robot, used by {@link BreakerRobotManager}. */
public class BreakerRobotConfig {

    private double secondsBetweenSelfChecks;
    private boolean autologNetworkTables;
    private boolean autoRegesterDevices;
    private BreakerFalconOrchestra orchestra;
    private boolean usesOrchestra;
    private BreakerAutoPath[] autoPaths;
    private boolean usesPaths;
    private BreakerRobotStartConfig startConfig;

    public BreakerRobotConfig(double secondsBetweenSelfChecks, boolean autoRegisterDevices, boolean autologNetworkTables, BreakerRobotStartConfig startConfig, BreakerFalconOrchestra orchestra,  BreakerAutoPath... autoPaths) {
        this.secondsBetweenSelfChecks = secondsBetweenSelfChecks;
        this.autologNetworkTables = autologNetworkTables;
        this.orchestra = orchestra;
        this.autoPaths = autoPaths;
        this.autoRegesterDevices = autoRegisterDevices;
        this.startConfig = startConfig;
        usesOrchestra = true;
        usesPaths = true;
    }


    public BreakerRobotConfig(double secondsBetweenSelfChecks, boolean autologNetworkTables, BreakerRobotStartConfig startConfig, BreakerFalconOrchestra orchestra) {
        this.secondsBetweenSelfChecks = secondsBetweenSelfChecks;
        this.autologNetworkTables = autologNetworkTables;
        this.orchestra = orchestra;
        this.autoPaths = new BreakerAutoPath[0];
        this.startConfig = startConfig;
        usesOrchestra = true;
        usesPaths = false;
    }

    public BreakerRobotConfig(double secondsBetweenSelfChecks, boolean autologNetworkTables, BreakerRobotStartConfig startConfig,  BreakerAutoPath... autoPaths) {
        this.secondsBetweenSelfChecks = secondsBetweenSelfChecks;
        this.autologNetworkTables = autologNetworkTables;
        this.orchestra = new BreakerFalconOrchestra();
        this.autoPaths = autoPaths;
        this.startConfig = startConfig;
        usesOrchestra = false;
        usesPaths = true;
    }

    public BreakerRobotConfig(double secondsBetweenSelfChecks, boolean autologNetworkTables, BreakerRobotStartConfig startConfig) {
        this.secondsBetweenSelfChecks = secondsBetweenSelfChecks;
        this.autologNetworkTables = autologNetworkTables;
        this.orchestra = new BreakerFalconOrchestra();
        this.autoPaths = new BreakerAutoPath[0];
        this.startConfig = startConfig;
        usesOrchestra = false;
        usesPaths = false;
    }

    public BreakerRobotConfig(BreakerRobotStartConfig startConfig, BreakerFalconOrchestra orchestra,  BreakerAutoPath... autoPaths) {
        this.secondsBetweenSelfChecks = 5;
        this.autologNetworkTables = false;
        this.autoRegesterDevices = true;
        this.orchestra = orchestra;
        this.autoPaths = autoPaths;
        this.startConfig = startConfig;
        usesOrchestra = true;
        usesPaths = true;
    }

    public BreakerRobotConfig(BreakerRobotStartConfig startConfig, BreakerFalconOrchestra orchestra) {
        this.secondsBetweenSelfChecks = 5;
        this.autologNetworkTables = false;
        this.autoRegesterDevices = true;
        this.orchestra = orchestra;
        this.autoPaths = new BreakerAutoPath[0];
        this.startConfig = startConfig;
        usesOrchestra = true;
        usesPaths = false;
    }

    public BreakerRobotConfig(BreakerRobotStartConfig startConfig, BreakerAutoPath... autoPaths) {
        this.secondsBetweenSelfChecks = 5;
        this.autologNetworkTables = false;
        this.autoRegesterDevices = true;
        this.orchestra = new BreakerFalconOrchestra();
        this.autoPaths = autoPaths;
        this.startConfig = startConfig;
        usesOrchestra = false;
        usesPaths = true;
    }

    public BreakerRobotConfig(BreakerRobotStartConfig startConfig) {
        this.secondsBetweenSelfChecks = 5;
        this.autologNetworkTables = false;
        this.autoRegesterDevices = true;
        this.orchestra = new BreakerFalconOrchestra();
        this.autoPaths = new BreakerAutoPath[0];
        this.startConfig = startConfig;
        usesOrchestra = false;
        usesPaths = false;
    }

    public BreakerRobotConfig() {
        this.secondsBetweenSelfChecks = 5;
        this.autologNetworkTables = false;
        this.autoRegesterDevices = true;
        this.orchestra = new BreakerFalconOrchestra();
        this.autoPaths = new BreakerAutoPath[0];
        this.startConfig = new BreakerRobotStartConfig();
        usesOrchestra = false;
        usesPaths = false;
    }

    public boolean getAutoRegesterDevices() {
        return autoRegesterDevices;
    }

    public BreakerAutoPath[] getAutoPaths() {
        return autoPaths;
    }

    public BreakerFalconOrchestra getOrchestra() {
        return orchestra;
    }

    public double getSecondsBetweenSelfChecks() {
        return secondsBetweenSelfChecks;
    }

    public boolean getAutologNetworkTables() {
        return autologNetworkTables;
    }

    public boolean UsesOrchestra() {
        return usesOrchestra;
    }

    public boolean UsesPaths() {
        return usesPaths;
    }

    public BreakerRobotStartConfig getStartConfig() {
        return startConfig;
    }

}
