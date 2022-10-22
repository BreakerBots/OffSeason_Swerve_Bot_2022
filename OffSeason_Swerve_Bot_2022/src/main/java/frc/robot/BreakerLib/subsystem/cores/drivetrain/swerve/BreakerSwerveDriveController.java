// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BreakerLib.driverstation.BreakerXboxController;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.util.math.mathfunctions.BreakerGenericMathFunction;

/** Controller object for the {@link BreakerSwerveDrive} drivetrain. */
public class BreakerSwerveDriveController extends CommandBase {

  private BreakerXboxController controller;
  private BreakerSwerveDrive baseDrivetrain;
  private BreakerGenericOdometer odometer;
  private boolean usesSuppliers, usesCurves, usesExternalOdmeter, turnOverride, linearOverride;
  private BreakerGenericMathFunction linearSpeedCurve, turnSpeedCurve;
  private DoubleSupplier forwardSpeedPercentSupplier, horizontalSpeedPercentSupplier, turnSpeedPercentSupplier, overrideForwardSupplier, overrideHorizontalSupplier, overrideTurnSupplier;

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
    usesExternalOdmeter = false;
    addRequirements(baseDrivetrain);
  }

  /**
   * Create a new BreakerSwerveDriveController using HID input and input curves.
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
    usesExternalOdmeter = false;
    addRequirements(baseDrivetrain);
  }

  /**
   * Creates a new BreakerSwerveDriveController using a BreakerSwerveDrive and
   * three DoubleSupplier objects.
   * 
   * @param baseDrivetrain                 The drive train used by this
   *                                       BreakerSwerveDriveController
   * @param forwardSpeedPrecentSupplier    The forward speed percent supplier
   * @param horizontalSpeedPrecentSupplier The horizontal speed percent supplier
   * @param turnSpeedPrecentSupplier       The turn speed percent supplier
   */
  public BreakerSwerveDriveController(BreakerSwerveDrive baseDrivetrain, DoubleSupplier forwardSpeedPrecentSupplier,
      DoubleSupplier horizontalSpeedPrecentSupplier, DoubleSupplier turnSpeedPrecentSupplier) {
    this.forwardSpeedPercentSupplier = forwardSpeedPrecentSupplier;
    this.horizontalSpeedPercentSupplier = horizontalSpeedPrecentSupplier;
    this.turnSpeedPercentSupplier = turnSpeedPrecentSupplier;
    this.baseDrivetrain = baseDrivetrain;
    usesSuppliers = true;
    usesCurves = false;
    usesExternalOdmeter = false;
    addRequirements(baseDrivetrain);
  }

  public BreakerSwerveDriveController(BreakerSwerveDrive baseDrivetrain, DoubleSupplier forwardSpeedPrecentSupplier,
      DoubleSupplier horizontalSpeedPrecentSupplier, DoubleSupplier turnSpeedPrecentSupplier,
      BreakerGenericMathFunction linearSpeedCurve, BreakerGenericMathFunction turnSpeedCurve) {
    this.forwardSpeedPercentSupplier = forwardSpeedPrecentSupplier;
    this.horizontalSpeedPercentSupplier = horizontalSpeedPrecentSupplier;
    this.turnSpeedPercentSupplier = turnSpeedPrecentSupplier;
    this.linearSpeedCurve = linearSpeedCurve;
    this.turnSpeedCurve = turnSpeedCurve;
    this.baseDrivetrain = baseDrivetrain;
    usesSuppliers = true;
    usesCurves = true;
    usesExternalOdmeter = false;
    addRequirements(baseDrivetrain);
  }

  /**
   * Makes a BreakerSwerveDriveController with an odometer, controller, and
   * drivetrain.
   * 
   * @param baseDrivetrain Swerve drivetrain.
   * @param odometer
   * @param controller
   */
  public BreakerSwerveDriveController(BreakerSwerveDrive baseDrivetrain, BreakerGenericOdometer odometer,
      BreakerXboxController controller) {
    this.controller = controller;
    this.baseDrivetrain = baseDrivetrain;
    this.odometer = odometer;
    usesSuppliers = false;
    usesCurves = false;
    usesExternalOdmeter = true;
    addRequirements(baseDrivetrain);
  }

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
    usesExternalOdmeter = true;
    addRequirements(baseDrivetrain);
  }

  public BreakerSwerveDriveController(BreakerSwerveDrive baseDrivetrain, BreakerGenericOdometer odometer,
      DoubleSupplier forwardSpeedPrecentSupplier, DoubleSupplier horizontalSpeedPrecentSupplier,
      DoubleSupplier turnSpeedPrecentSupplier) {
    this.forwardSpeedPercentSupplier = forwardSpeedPrecentSupplier;
    this.horizontalSpeedPercentSupplier = horizontalSpeedPrecentSupplier;
    this.turnSpeedPercentSupplier = turnSpeedPrecentSupplier;
    this.baseDrivetrain = baseDrivetrain;
    this.odometer = odometer;
    usesSuppliers = true;
    usesCurves = false;
    usesExternalOdmeter = true;
    addRequirements(baseDrivetrain);
  }

  public BreakerSwerveDriveController(BreakerSwerveDrive baseDrivetrain, BreakerGenericOdometer odometer,
      DoubleSupplier forwardSpeedPrecentSupplier, DoubleSupplier horizontalSpeedPrecentSupplier,
      DoubleSupplier turnSpeedPrecentSupplier, BreakerGenericMathFunction linearSpeedCurve,
      BreakerGenericMathFunction turnSpeedCurve) {
    this.forwardSpeedPercentSupplier = forwardSpeedPrecentSupplier;
    this.horizontalSpeedPercentSupplier = horizontalSpeedPrecentSupplier;
    this.turnSpeedPercentSupplier = turnSpeedPrecentSupplier;
    this.linearSpeedCurve = linearSpeedCurve;
    this.turnSpeedCurve = turnSpeedCurve;
    this.baseDrivetrain = baseDrivetrain;
    this.odometer = odometer;
    usesSuppliers = true;
    usesCurves = true;
    usesExternalOdmeter = true;
    addRequirements(baseDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = 0.0;
    double horizontal = 0.0;
    double turn = 0.0;
    /** If double suppliers are used */
    if (usesSuppliers) {
      forward = !linearOverride ? forwardSpeedPercentSupplier.getAsDouble() : overrideForwardSupplier.getAsDouble();
      horizontal = !linearOverride ? horizontalSpeedPercentSupplier.getAsDouble() : overrideForwardSupplier.getAsDouble();
      turn = !turnOverride ? turnSpeedPercentSupplier.getAsDouble() : overrideTurnSupplier.getAsDouble();
    } else { /** Use controller inputs */
      forward = !linearOverride ? -controller.getLeftY() : overrideForwardSupplier.getAsDouble();
      horizontal = !linearOverride ? -controller.getLeftX() : overrideHorizontalSupplier.getAsDouble();
      turn = !turnOverride ? controller.getRightX() :  overrideTurnSupplier.getAsDouble();
    }

    if (usesCurves && !(linearOverride || turnOverride)) {
      forward = linearSpeedCurve.getSignRelativeValueAtX(forward);
      horizontal = linearSpeedCurve.getSignRelativeValueAtX(horizontal);
      turn = turnSpeedCurve.getSignRelativeValueAtX(turn);
    }

    if (usesExternalOdmeter) {
      baseDrivetrain.moveWithPercentInputRelativeToField(MathUtil.clamp(forward, -1.0, 1.0),
          MathUtil.clamp(horizontal, -1.0, 1.0), MathUtil.clamp(turn, -1.0, 1.0), odometer);
    } else {
      baseDrivetrain.moveWithPercentInput(MathUtil.clamp(forward, -1.0, 1.0),
          MathUtil.clamp(horizontal, -1.0, 1.0), MathUtil.clamp(turn, -1.0, 1.0));
    }
  }

  public void overrideTurnInput(DoubleSupplier turnSupplier) {
    turnOverride = true;
    overrideTurnSupplier = turnSupplier;
  }

  public void overrideLinearInput(DoubleSupplier forwardSupplier, DoubleSupplier horizontalSupplier) {
    linearOverride = true;
    overrideHorizontalSupplier = horizontalSupplier;
    overrideForwardSupplier = forwardSupplier;
  }

  public void overrideAllInputs(DoubleSupplier forwardSupplier, DoubleSupplier horizontalSupplier, DoubleSupplier turnSupplier) {
    overrideTurnInput(turnSupplier);
    overrideLinearInput(horizontalSupplier, forwardSupplier);
  }

  public void endLinearOverride() {
    linearOverride = false;
  }

  public void endTurnOverride() {
    turnOverride = false;
  }

  public void endAllOverrides() {
    endLinearOverride();
    endTurnOverride();
  }

  public boolean isLinearInputOverridden() {
    return linearOverride;
  }

  public boolean isTurnInputOverridden() {
    return turnOverride;
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
