// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.differential;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BreakerLib.driverstation.BreakerXboxController;
import frc.robot.BreakerLib.util.math.polynomials.BreakerGenericPolynomial;

public class BreakerDiffDriveController extends CommandBase {
  
  private BreakerXboxController controller;
  private BreakerDiffDrive baseDrivetrain;
  private boolean usesSuppliers, usesCurves;
  private BreakerGenericPolynomial netSpeedCurve, turnSpeedCurve;
  private DoubleSupplier netSpeedPrecentSupplier, turnSpeedPrecentSupplier;
  public BreakerDiffDriveController(BreakerDiffDrive baseDrivetrain, BreakerXboxController controller) {
    this.controller = controller;
    this.baseDrivetrain = baseDrivetrain;
    usesSuppliers = false;
    usesCurves = false;
    addRequirements(baseDrivetrain);
  }

  public BreakerDiffDriveController(BreakerDiffDrive baseDrivetrain, BreakerXboxController controller, BreakerGenericPolynomial netSpeedCurve, BreakerGenericPolynomial turnSpeedCurve) {
    this.controller = controller;
    this.netSpeedCurve = netSpeedCurve;
    this.turnSpeedCurve = turnSpeedCurve;
    this.baseDrivetrain = baseDrivetrain;
    usesSuppliers = false;
    usesCurves = true;
    addRequirements(baseDrivetrain);
  }

  public BreakerDiffDriveController(BreakerDiffDrive baseDrivetrain, DoubleSupplier netSpeedPrecentSupplier, DoubleSupplier turnSpeedPrecentSupplier) {
    this.netSpeedPrecentSupplier = netSpeedPrecentSupplier;
    this.turnSpeedPrecentSupplier = turnSpeedPrecentSupplier;
    this.baseDrivetrain = baseDrivetrain;
    usesSuppliers = true;
    usesCurves = false;
    addRequirements(baseDrivetrain);
  }

  public BreakerDiffDriveController(BreakerDiffDrive baseDrivetrain, DoubleSupplier netSpeedPrecentSupplier, DoubleSupplier turnSpeedPrecentSupplier, BreakerGenericPolynomial netSpeedCurve, BreakerGenericPolynomial turnSpeedCurve) {
    this.netSpeedPrecentSupplier = netSpeedPrecentSupplier;
    this.turnSpeedPrecentSupplier = turnSpeedPrecentSupplier;
    this.netSpeedCurve = netSpeedCurve;
    this.turnSpeedCurve = turnSpeedCurve;
    this.baseDrivetrain = baseDrivetrain;
    usesSuppliers = true;
    usesCurves = true;
    addRequirements(baseDrivetrain);
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
      net = controller.getBaseController().getRightTriggerAxis() - controller.getBaseController().getLeftTriggerAxis();
      turn = controller.getBaseController().getLeftX();
    }
    if (usesCurves) {
      net = netSpeedCurve.getSignRelativeValueAtX(net);
      turn = turnSpeedCurve.getSignRelativeValueAtX(turn);
    }
    baseDrivetrain.arcadeDrive(MathUtil.clamp(net, -1.0, 1.0), MathUtil.clamp(turn, -1.0, 1.0));
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
