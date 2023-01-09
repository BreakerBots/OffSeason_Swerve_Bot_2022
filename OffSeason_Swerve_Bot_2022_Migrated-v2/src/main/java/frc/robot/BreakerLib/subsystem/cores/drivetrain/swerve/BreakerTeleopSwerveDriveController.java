// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerGenericGamepad;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.util.math.functions.BreakerGenericMathFunction;

/** Controller object for the {@link BreakerSwerveDrive} drivetrain. */
public class BreakerTeleopSwerveDriveController extends CommandBase {

  private BreakerGenericGamepad controller;
  private BreakerSwerveDrive baseDrivetrain;
  private BreakerGenericOdometer odometer;
  private boolean usesSuppliers, usesCurves, usesExternalOdometer, usesRateLimiters, turnOverride, linearOverride;
  private BreakerGenericMathFunction linearSpeedCurve, turnSpeedCurve;
  private SlewRateLimiter forwardRateLimiter, horizontalRateLimiter, turnRateLimiter;
  private DoubleSupplier forwardSpeedPercentSupplier, horizontalSpeedPercentSupplier, turnSpeedPercentSupplier,
      overrideForwardSupplier, overrideHorizontalSupplier, overrideTurnSupplier;

  /**
   * Creates a BreakerSwerveDriveController which only utilizes HID input.
   * 
   * @param baseDrivetrain Swerve drivetrain.
   * @param controller     Xbox controller.
   */
  public BreakerTeleopSwerveDriveController(BreakerSwerveDrive baseDrivetrain, BreakerXboxController controller) {
    this.controller = controller;
    this.baseDrivetrain = baseDrivetrain;
    usesSuppliers = false;
    usesCurves = false;
    usesExternalOdometer = false;
    usesRateLimiters = false;
    addRequirements(baseDrivetrain);
  }

  /**
   * Creates a new BreakerSwerveDriveController which uses percent speed values.
   * 
   * @param baseDrivetrain                 The drive train used by this
   *                                       BreakerSwerveDriveController.
   * @param forwardSpeedPercentSupplier    The forward speed percent supplier.
   * @param horizontalSpeedPercentSupplier The horizontal speed percent supplier.
   * @param turnSpeedPercentSupplier       The turn speed percent supplier.
   */
  public BreakerTeleopSwerveDriveController(BreakerSwerveDrive baseDrivetrain, DoubleSupplier forwardSpeedPercentSupplier,
      DoubleSupplier horizontalSpeedPercentSupplier, DoubleSupplier turnSpeedPercentSupplier) {
    this.forwardSpeedPercentSupplier = forwardSpeedPercentSupplier;
    this.horizontalSpeedPercentSupplier = horizontalSpeedPercentSupplier;
    this.turnSpeedPercentSupplier = turnSpeedPercentSupplier;
    this.baseDrivetrain = baseDrivetrain;
    usesSuppliers = true;
    usesCurves = false;
    usesExternalOdometer = false;
    usesRateLimiters = false;
    addRequirements(baseDrivetrain);
  }

  /**
   * Makes a BreakerSwerveDriveController which uses an odometer and HID input.
   * 
   * @param baseDrivetrain Swerve drivetrain.
   * @param odometer       Odometer to use.
   * @param controller     Xbox controller to provide input.
   */
  public BreakerTeleopSwerveDriveController(BreakerSwerveDrive baseDrivetrain, BreakerGenericOdometer odometer,
      BreakerGenericGamepad controller) {
    this.controller = controller;
    this.baseDrivetrain = baseDrivetrain;
    this.odometer = odometer;
    usesSuppliers = false;
    usesCurves = false;
    usesExternalOdometer = true;
    usesRateLimiters = false;
    addRequirements(baseDrivetrain);
  }

  /**
   * Makes a BreakerSwerveDriveController which uses an odometer and percent speed
   * values.
   * 
   * @param baseDrivetrain                 Swerve drivetrain.
   * @param odometer                       Odometer to use.
   * @param forwardSpeedPercentSupplier    The forward speed percent supplier.
   * @param horizontalSpeedPercentSupplier The horizontal speed percent supplier.
   * @param turnSpeedPercentSupplier       The turn speed percent supplier.
   */
  public BreakerTeleopSwerveDriveController(BreakerSwerveDrive baseDrivetrain, BreakerGenericOdometer odometer,
      DoubleSupplier forwardSpeedPercentSupplier, DoubleSupplier horizontalSpeedPercentSupplier,
      DoubleSupplier turnSpeedPercentSupplier) {
    this.forwardSpeedPercentSupplier = forwardSpeedPercentSupplier;
    this.horizontalSpeedPercentSupplier = horizontalSpeedPercentSupplier;
    this.turnSpeedPercentSupplier = turnSpeedPercentSupplier;
    this.baseDrivetrain = baseDrivetrain;
    this.odometer = odometer;
    usesSuppliers = true;
    usesCurves = false;
    usesExternalOdometer = true;
    usesRateLimiters = false;
    addRequirements(baseDrivetrain);
  }

  public BreakerTeleopSwerveDriveController addSlewRateLimiters(SlewRateLimiter forwardRateLimiter, SlewRateLimiter  horizontalRateLimiter, SlewRateLimiter turnRateLimiter) {
    this.forwardRateLimiter = forwardRateLimiter;
    this.horizontalRateLimiter = horizontalRateLimiter;
    this.turnRateLimiter = turnRateLimiter;
    usesRateLimiters = true;
    return this;
  }

  public BreakerTeleopSwerveDriveController addSpeedCurves(BreakerGenericMathFunction linearSpeedCurve, BreakerGenericMathFunction turnSpeedCurve) {
    this.linearSpeedCurve = linearSpeedCurve;
    this.turnSpeedCurve = turnSpeedCurve;
    usesCurves = true;
    return this;
  }

  /**
   * Overrides turn input with selected percent values.
   * 
   * @param turnSupplier Turn speed percent supplier.
   */
  public void overrideTurnInput(DoubleSupplier turnSupplier) {
    turnOverride = true;
    overrideTurnSupplier = turnSupplier;
  }

  /**
   * Overrides linear inputs with selected percent values.
   * 
   * @param forwardSupplier    Forward speed percent supplier.
   * @param horizontalSupplier Horizontal speed percent supplier.
   */
  public void overrideLinearInput(DoubleSupplier forwardSupplier, DoubleSupplier horizontalSupplier) {
    linearOverride = true;
    overrideHorizontalSupplier = horizontalSupplier;
    overrideForwardSupplier = forwardSupplier;
  }

  /**
   * Overrides linear and turn inputs with selected percent suppliers.
   * 
   * @param forwardSupplier    Forward speed percent supplier.
   * @param horizontalSupplier Horizontal speed percent supplier.
   * @param turnSupplier       Turn speed percent supplier.
   */
  public void overrideAllInputs(DoubleSupplier forwardSupplier, DoubleSupplier horizontalSupplier,
      DoubleSupplier turnSupplier) {
    overrideTurnInput(turnSupplier);
    overrideLinearInput(horizontalSupplier, forwardSupplier);
  }

  /** Disables override of linear drive input with percent suppliers. */
  public void endLinearOverride() {
    linearOverride = false;
  }

  /** Disables override of rotation input with a percent supplier. */
  public void endTurnOverride() {
    turnOverride = false;
  }

  /**
   * Disables override of rotation input and linear input with percent suppliers.
   */
  public void endAllOverrides() {
    endLinearOverride();
    endTurnOverride();
  }

  /** @return If linear input is being overwritten. */
  public boolean isLinearInputOverridden() {
    return linearOverride;
  }

  /** @return If turn input is being overwritten. */
  public boolean isTurnInputOverridden() {
    return turnOverride;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    double forward = 0.0;
    double horizontal = 0.0;
    double turn = 0.0;

    if (usesSuppliers) { // If double suppliers are used.
      // Default suppliers are used unless overwritten.
      forward = forwardSpeedPercentSupplier.getAsDouble();
      horizontal = horizontalSpeedPercentSupplier.getAsDouble();
      turn = turnSpeedPercentSupplier.getAsDouble();
    } else { // Use controller inputs.
      // Controller inputs are used unless overwritten.
      forward = controller.getLeftThumbstick().getY();
      horizontal = controller.getLeftThumbstick().getX();
      turn = controller.getRightThumbstick().getX();
    }

    // Speed curves are applied if overrides are not active.
    if (usesCurves) {
      forward = linearSpeedCurve.getSignRelativeValueAtX(forward);
      horizontal = linearSpeedCurve.getSignRelativeValueAtX(horizontal);
      turn = turnSpeedCurve.getSignRelativeValueAtX(turn);
    }

    if (usesRateLimiters) {
      forward = forwardRateLimiter.calculate(forward);
      horizontal = horizontalRateLimiter.calculate(horizontal);
      turn = turnRateLimiter.calculate(turn);
    }

    if (linearOverride) {
      forward = overrideForwardSupplier.getAsDouble();
      horizontal = overrideHorizontalSupplier.getAsDouble();
    }

    if (turnOverride) {
      turn = overrideTurnSupplier.getAsDouble();
    }

    // Movement relative to field.
    if (usesExternalOdometer) {
      // External odometry source is used.
      baseDrivetrain.moveWithPercentInputRelativeToField(MathUtil.clamp(forward, -1.0, 1.0),
          MathUtil.clamp(horizontal, -1.0, 1.0), MathUtil.clamp(turn, -1.0, 1.0), odometer);
    } else {
      // Swerve drive's own odometry is used.
      baseDrivetrain.moveWithPercentInputRelativeToField(MathUtil.clamp(forward, -1.0, 1.0),
          MathUtil.clamp(horizontal, -1.0, 1.0), MathUtil.clamp(turn, -1.0, 1.0));
    }

    SmartDashboard.putString("Drive input", String.format("\nF: %.2f, H: %.2f, T: %.2f", forward, horizontal, turn));
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
