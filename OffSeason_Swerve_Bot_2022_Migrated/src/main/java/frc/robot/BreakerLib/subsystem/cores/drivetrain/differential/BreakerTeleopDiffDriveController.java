// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.differential;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerGenericGamepad;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXinput;
import frc.robot.BreakerLib.util.math.functions.BreakerGenericMathFunction;

public class BreakerTeleopDiffDriveController extends CommandBase {
  
  private BreakerGenericGamepad controller;
  private BreakerDiffDrive baseDrivetrain;
  private boolean usesSuppliers, usesCurves, usesRateLimiters, inputOverride;
  private BreakerGenericMathFunction netSpeedCurve, turnSpeedCurve;
  private SlewRateLimiter netRateLimiter, turnRateLimiter;
  private DoubleSupplier netSpeedPrecentSupplier, turnSpeedPrecentSupplier, overrideNetSup, overrideTurnSup;
  public BreakerTeleopDiffDriveController(BreakerDiffDrive baseDrivetrain, BreakerGenericGamepad controller) {
    this.controller = controller;
    this.baseDrivetrain = baseDrivetrain;
    usesSuppliers = false;
    usesCurves = false;
    usesRateLimiters = false;
    inputOverride = false;
    addRequirements(baseDrivetrain);
  }

  public BreakerTeleopDiffDriveController(BreakerDiffDrive baseDrivetrain, DoubleSupplier netSpeedPrecentSupplier, DoubleSupplier turnSpeedPrecentSupplier) {
    this.netSpeedPrecentSupplier = netSpeedPrecentSupplier;
    this.turnSpeedPrecentSupplier = turnSpeedPrecentSupplier;
    this.baseDrivetrain = baseDrivetrain;
    usesSuppliers = true;
    usesCurves = false;
    usesRateLimiters = false;
    inputOverride = false;
    addRequirements(baseDrivetrain);
  }

  public BreakerTeleopDiffDriveController addSpeedCurves(BreakerGenericMathFunction netSpeedCurve, BreakerGenericMathFunction turnSpeedCurve) {
    this.netSpeedCurve = netSpeedCurve;
    this.turnSpeedCurve = turnSpeedCurve;
    usesCurves = true;
    return this;
  }

  public BreakerTeleopDiffDriveController addRateLimiters(SlewRateLimiter netSpeedLimiter, SlewRateLimiter turnSpeedLimiter) {
    this.netRateLimiter = netSpeedLimiter;
    this.netRateLimiter = netSpeedLimiter;
    usesRateLimiters = true;
    return this;
  }

  public void overrideAllInputs(DoubleSupplier netSpeedOverridePrecentSupplier, DoubleSupplier turnSpeedOverridePrecentSupplier) {
    overrideNetSup = netSpeedOverridePrecentSupplier;
    overrideTurnSup = turnSpeedOverridePrecentSupplier;
    inputOverride = true;
  }

  public void endAllOverrides() {
    inputOverride = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double net = 0.0;
    double turn = 0.0;
    if (usesSuppliers) {
      net = netSpeedPrecentSupplier.getAsDouble();
      turn = turnSpeedPrecentSupplier.getAsDouble();
    } else {
      net = controller.getRightTrigger().get() - controller.getRightTrigger().get();
      turn = controller.getLeftThumbstick().getX();
    }

    if (usesCurves) {
      net = netSpeedCurve.getSignRelativeValueAtX(net);
      turn = turnSpeedCurve.getSignRelativeValueAtX(turn);
    }

    if (usesRateLimiters) {
      net = netRateLimiter.calculate(net);
      turn = turnRateLimiter.calculate(turn);
    }

    if (inputOverride) {
      net = overrideNetSup.getAsDouble();
      turn = overrideTurnSup.getAsDouble();
    }

    baseDrivetrain.arcadeDrive(MathUtil.clamp(net, -1.0, 1.0), MathUtil.clamp(turn, -1.0, 1.0));
  }

  public void overrideInputs(DoubleSupplier netSpeedSupplier, DoubleSupplier turnSpeedSupplier) {
    inputOverride = true;
    overrideNetSup = netSpeedSupplier;
    overrideTurnSup = turnSpeedSupplier;
  }

  public void endInputOverride() {
    inputOverride = false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
