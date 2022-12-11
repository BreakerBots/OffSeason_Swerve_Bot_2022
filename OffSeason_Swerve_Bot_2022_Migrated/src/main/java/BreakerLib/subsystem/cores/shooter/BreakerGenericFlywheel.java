// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package BreakerLib.subsystem.cores.shooter;

import BreakerLib.devices.BreakerGenericLoopedDevice;
import BreakerLib.util.BreakerArbitraryFeedforwardProvider;
import BreakerLib.util.logging.BreakerLog;
import BreakerLib.util.math.BreakerMath;
import BreakerLib.util.test.suites.BreakerGenericTestSuiteImplementation;
import BreakerLib.util.test.suites.flywheel.BreakerFlywheelTestSuite;

/** A class representing a robot's shooter flywheel and its assocated controle loop */
public abstract class BreakerGenericFlywheel extends BreakerGenericLoopedDevice implements BreakerGenericTestSuiteImplementation<BreakerFlywheelTestSuite> {
    protected BreakerFlywheelConfig config;
    protected double flywheelTargetRPM = 0;
    protected BreakerFlywheelTestSuite testSuite;
    protected double lastVel = 0;
    protected double accel = 0;
    protected double accelTol;
    protected double velTol;
    protected BreakerArbitraryFeedforwardProvider ffProvider;
    

    public BreakerGenericFlywheel(BreakerFlywheelConfig config) {
        this.config = config;
        testSuite = new BreakerFlywheelTestSuite(this);
        velTol = config.getVelocityTolerence();
        accelTol = config.getAcclerationTolerence();
        ffProvider = config.getArbFFProvider();
    }

    public void setFlywheelSpeed(double flywheelTargetSpeedRPM) {
        flywheelTargetRPM = flywheelTargetSpeedRPM;
    }

    public abstract double getFlywheelVelRSU();

    public abstract double getFlywheelRPM();

    public double getFlywheelTargetRPM() {
        return flywheelTargetRPM;
    }

    /** sets flywheel speed to 0 RPM */
    public void stopFlywheel() {
        setFlywheelSpeed(0);
        BreakerLog.logSuperstructureEvent("flywheel stoped");
    }


    protected abstract void runFlywheel();

    public boolean flywheelIsAtTargetVel() {
        return BreakerMath.lambdaEquals(flywheelTargetRPM, getFlywheelRPM(), velTol) && BreakerMath.lambdaEquals(accel, 0, accelTol);
    }

    @Override
    public void periodic() {
        runFlywheel();
        accel = getFlywheelRPM() - lastVel;
        lastVel = getFlywheelRPM();
    }

    @Override
    public BreakerFlywheelTestSuite getTestSuite() {
        return testSuite;
    }
}
