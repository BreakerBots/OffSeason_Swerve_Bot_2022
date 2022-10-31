// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BreakerLib.driverstation.BreakerXboxController;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.util.math.functions.BreakerGenericMathFunction;

/** Controller object for the {@link BreakerSwerveDrive} drivetrain. */
public class BreakerSwerveDriveController extends CommandBase {

  private BreakerXboxController controller;
  private BreakerSwerveDrive baseDrivetrain;
  private BreakerGenericOdometer odometer;
  private boolean usesSuppliers, usesCurves, usesExternalOdometer, turnOverride, linearOverride;
  private BreakerGenericMathFunction linearSpeedCurve, turnSpeedCurve;
  private DoubleSupplier forwardSpeedPercentSupplier, horizontalSpeedPercentSupplier, turnSpeedPercentSupplier,
      overrideForwardSupplier, overrideHorizontalSupplier, overrideTurnSupplier;

  /**
   * Creates a BreakerSwerveDriveController which only utilizes HID input.
   * 
   * @param baseDrivetrain Swerve drivetrain.
   * @param controller     Xbox controller.
   */
  public BreakerSwerveDriveController(BreakerSwerveDrive baseDrivetrain, BreakerXboxController controller) {
    this.controller = controller;
    this.baseDrivetrain = baseDrivetrain;
    usesSuppliers = false;
    usesCurves = false;
    usesExternalOdometer = false;
    addRequirements(baseDrivetrain);
  }

  /**
   * Create a new BreakerSwerveDriveController which uses HID input and input
   * curves.
   * 
   * @param baseDrivetrain   Swerve drivetrain.
   * @param controller       Xbox controller.
   * @param linearSpeedCurve Linear speed curve.
   * @param turnSpeedCurve   Turn speed curve.
   */
  public BreakerSwerveDriveController(BreakerSwerveDrive baseDrivetrain, BreakerXboxController controller,
      BreakerGenericMathFunction linearSpeedCurve, BreakerGenericMathFunction turnSpeedCurve) {
    this.controller = controller;
    this.linearSpeedCurve = linearSpeedCurve;
    this.turnSpeedCurve = turnSpeedCurve;
    this.baseDrivetrain = baseDrivetrain;
    usesSuppliers = false;
    usesCurves = true;
    usesExternalOdometer = false;
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
  public BreakerSwerveDriveController(BreakerSwerveDrive baseDrivetrain, DoubleSupplier forwardSpeedPercentSupplier,
      DoubleSupplier horizontalSpeedPercentSupplier, DoubleSupplier turnSpeedPercentSupplier) {
    this.forwardSpeedPercentSupplier = forwardSpeedPercentSupplier;
    this.horizontalSpeedPercentSupplier = horizontalSpeedPercentSupplier;
    this.turnSpeedPercentSupplier = turnSpeedPercentSupplier;
    this.baseDrivetrain = baseDrivetrain;
    usesSuppliers = true;
    usesCurves = false;
    usesExternalOdometer = false;
    addRequirements(baseDrivetrain);
  }

  /**
   * Creates a new BreakerSwerveDriveController which uses percent speed values
   * and input curves
   * 
   * @param baseDrivetrain                 The drive train used by this
   *                                       BreakerSwerveDriveController.
   * @param forwardSpeedPercentSupplier    The forward speed percent supplier.
   * @param horizontalSpeedPercentSupplier The horizontal speed percent supplier.
   * @param turnSpeedPercentSupplier       The turn speed percent supplier.
   * @param linearSpeedCurve               Linear speed curve.
   * @param turnSpeedCurve                 Rotation speed curve.
   */
  public BreakerSwerveDriveController(BreakerSwerveDrive baseDrivetrain, DoubleSupplier forwardSpeedPercentSupplier,
      DoubleSupplier horizontalSpeedPercentSupplier, DoubleSupplier turnSpeedPercentSupplier,
      BreakerGenericMathFunction linearSpeedCurve, BreakerGenericMathFunction turnSpeedCurve) {
    this.forwardSpeedPercentSupplier = forwardSpeedPercentSupplier;
    this.horizontalSpeedPercentSupplier = horizontalSpeedPercentSupplier;
    this.turnSpeedPercentSupplier = turnSpeedPercentSupplier;
    this.linearSpeedCurve = linearSpeedCurve;
    this.turnSpeedCurve = turnSpeedCurve;
    this.baseDrivetrain = baseDrivetrain;
    usesSuppliers = true;
    usesCurves = true;
    usesExternalOdometer = false;
    addRequirements(baseDrivetrain);
  }

  /**
   * Makes a BreakerSwerveDriveController which uses an odometer and HID input.
   * 
   * @param baseDrivetrain Swerve drivetrain.
   * @param odometer       Odometer to use.
   * @param controller     Xbox controller to provide input.
   */
  public BreakerSwerveDriveController(BreakerSwerveDrive baseDrivetrain, BreakerGenericOdometer odometer,
      BreakerXboxController controller) {
    this.controller = controller;
    this.baseDrivetrain = baseDrivetrain;
    this.odometer = odometer;
    usesSuppliers = false;
    usesCurves = false;
    usesExternalOdometer = true;
    addRequirements(baseDrivetrain);
  }

  /**
   * Makes a BreakerSwerveDriveController which uses an odometer, HID input, and
   * speed curves.
   * 
   * @param baseDrivetrain   Swerve drivetrain.
   * @param odometer         Odometer to use.
   * @param controller       Xbox controller to provide input.
   * @param linearSpeedCurve Linear speed curve.
   * @param turnSpeedCurve   Rotation speed curve.
   */
  public BreakerSwerveDriveController(BreakerSwerveDrive baseDrivetrain, BreakerGenericOdometer odometer,
      BreakerXboxController controller, BreakerGenericMathFunction linearSpeedCurve,
      BreakerGenericMathFunction turnSpeedCurve) {
    this.controller = controller;
    this.linearSpeedCurve = linearSpeedCurve;
    this.turnSpeedCurve = turnSpeedCurve;
    this.baseDrivetrain = baseDrivetrain;
    this.odometer = odometer;
    usesSuppliers = false;
    usesCurves = true;
    usesExternalOdometer = true;
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
  public BreakerSwerveDriveController(BreakerSwerveDrive baseDrivetrain, BreakerGenericOdometer odometer,
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
    addRequirements(baseDrivetrain);
  }

  /**
   * Makes a BreakerSwerveDriveController which uses an odometer, percent speed
   * values, and speed curves.
   * 
   * @param baseDrivetrain                 Swerve drivetrain.
   * @param odometer                       Odometer to use.
   * @param forwardSpeedPercentSupplier    The forward speed percent supplier.
   * @param horizontalSpeedPercentSupplier The horizontal speed percent supplier.
   * @param turnSpeedPercentSupplier       The turn speed percent supplier.
   * @param linearSpeedCurve               Linear speed curve.
   * @param turnSpeedCurve                 Rotation speed curve.
   */
  public BreakerSwerveDriveController(BreakerSwerveDrive baseDrivetrain, BreakerGenericOdometer odometer,
      DoubleSupplier forwardSpeedPercentSupplier, DoubleSupplier horizontalSpeedPercentSupplier,
      DoubleSupplier turnSpeedPercentSupplier, BreakerGenericMathFunction linearSpeedCurve,
      BreakerGenericMathFunction turnSpeedCurve) {
    this.forwardSpeedPercentSupplier = forwardSpeedPercentSupplier;
    this.horizontalSpeedPercentSupplier = horizontalSpeedPercentSupplier;
    this.turnSpeedPercentSupplier = turnSpeedPercentSupplier;
    this.linearSpeedCurve = linearSpeedCurve;
    this.turnSpeedCurve = turnSpeedCurve;
    this.baseDrivetrain = baseDrivetrain;
    this.odometer = odometer;
    usesSuppliers = true;
    usesCurves = true;
    usesExternalOdometer = true;
    addRequirements(baseDrivetrain);
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
      forward = !linearOverride ? forwardSpeedPercentSupplier.getAsDouble() : overrideForwardSupplier.getAsDouble();
      horizontal = !linearOverride ? horizontalSpeedPercentSupplier.getAsDouble()
          : overrideForwardSupplier.getAsDouble();
      turn = !turnOverride ? turnSpeedPercentSupplier.getAsDouble() : overrideTurnSupplier.getAsDouble();
    } else { // Use controller inputs.
      // Controller inputs are used unless overwritten.
      forward = !linearOverride ? -controller.getLeftY() : overrideForwardSupplier.getAsDouble();
      horizontal = !linearOverride ? -controller.getLeftX() : overrideHorizontalSupplier.getAsDouble();
      turn = !turnOverride ? controller.getRightX() : overrideTurnSupplier.getAsDouble();
    }

    // Speed curves are applied if overrides are not active.
    if (usesCurves && !(linearOverride || turnOverride)) {
      forward = linearSpeedCurve.getSignRelativeValueAtX(forward);
      horizontal = linearSpeedCurve.getSignRelativeValueAtX(horizontal);
      turn = turnSpeedCurve.getSignRelativeValueAtX(turn);
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
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
