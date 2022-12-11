// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package BreakerLib.util.test.suites.flywheel;

import java.util.Currency;

import BreakerLib.subsystem.cores.shooter.BreakerFalconFlywheel;
import BreakerLib.subsystem.cores.shooter.BreakerGenericFlywheel;
import BreakerLib.util.math.averages.BreakerAverage;
import BreakerLib.util.test.suites.BreakerTestBase;
import BreakerLib.util.test.suites.BreakerTestSuiteDataLogType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BreakerRepeatedFlywheelChargeCycleTest extends BreakerTestBase {
  /** Creates a new RepeatedFlywheelSpinUpTest. */
  private int numberOfCycles, curPhase;
  private double targetRPM, phaseStartTime;
  private BreakerGenericFlywheel baseFlywheel;
  private boolean isSpinUp;
  private BreakerAverage avgChargeUp, avgSpinDown;
  public BreakerRepeatedFlywheelChargeCycleTest(int numberOfCycles, double targetRPM, BreakerGenericFlywheel baseFlywheel, BreakerTestSuiteDataLogType logType) {
    super(logType, "Repeated_Flywheel_Charge_Cycle_Test", "Cycles: " + numberOfCycles);
    this.numberOfCycles = numberOfCycles;
    this.targetRPM = targetRPM;
    this.baseFlywheel = baseFlywheel;
    avgChargeUp = new BreakerAverage();
    avgSpinDown = new BreakerAverage();
    curPhase = 0;
    phaseStartTime = 0;
    isSpinUp = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    baseFlywheel.setFlywheelSpeed(targetRPM);
    phaseStartTime = Timer.getFPGATimestamp();
    isSpinUp = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isSpinUp && baseFlywheel.flywheelIsAtTargetVel()) {
      double time = Timer.getFPGATimestamp();
      avgChargeUp.addValue(time - phaseStartTime);
      phaseStartTime = time;
      isSpinUp = false;
      baseFlywheel.stopFlywheel();
      periodicLog("SPINUP - RPM: "+baseFlywheel.getFlywheelRPM());
      curPhase ++;
    } else if (!isSpinUp && (baseFlywheel.getFlywheelRPM() == 0)) {
      double time = Timer.getFPGATimestamp();
      avgSpinDown.addValue(time - phaseStartTime);
      phaseStartTime = time;
      isSpinUp = true;
      baseFlywheel.setFlywheelSpeed(targetRPM);
      curPhase ++;
    }
    periodicLog((isSpinUp ? "SPIN-UP" : "SPIN-DOWN") + "- RPM: " + baseFlywheel.getFlywheelRPM());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return curPhase == (numberOfCycles * 2);
  }
}
