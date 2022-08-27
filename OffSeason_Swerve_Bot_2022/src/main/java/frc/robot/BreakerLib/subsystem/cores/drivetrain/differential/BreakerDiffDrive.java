// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.differential;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.BreakerLib.devices.sensors.BreakerPigeon2;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.position.odometry.differential.BreakerDiffDriveState;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain;
import frc.robot.BreakerLib.util.BreakerCTREUtil;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.BreakerUnits;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/** Differential drivetrain that uses Falcon 500 motors. */
public class BreakerDiffDrive extends BreakerGenericDrivetrain {
  private WPI_TalonFX leftLead;
  private WPI_TalonFX[] leftMotors;
  private MotorControllerGroup leftDrive;

  private WPI_TalonFX rightLead;
  private WPI_TalonFX[] rightMotors;
  private MotorControllerGroup rightDrive;

  private DifferentialDrive diffDrive;
  private BreakerDiffDriveConfig driveConfig;

  private BreakerPigeon2 pigeon2;
  private DifferentialDriveOdometry driveOdometer;
  private BreakerMovementState2d prevMovementState = new BreakerMovementState2d();
  private BreakerMovementState2d curMovementState = new BreakerMovementState2d();
  private double prevOdometryUpdateTimestamp = 0;

  private boolean invertL;
  private boolean invertR;

  private boolean isAutoPowerManaged = true;
  private DevicePowerMode powerMode = DevicePowerMode.FULL_POWER_MODE;

  /**
   * Creates a new BreakerDiffDrive.
   * 
   * @param leftMotors  Array of left Falcon 500 motors.
   * @param rightMotors Array of right Falcon 500 motors.
   * @param invertL     Invert left side of drivetrain.
   * @param invertR     Invert right side of drivetrain.
   * @param pigeon2     Pigeon 2 IMU.
   * @param driveConfig Config for drivetrain.
   */
  public BreakerDiffDrive(WPI_TalonFX[] leftMotors, WPI_TalonFX[] rightMotors, boolean invertL, boolean invertR,
      BreakerPigeon2 pigeon2, BreakerDiffDriveConfig driveConfig) {

    // Left motors.
    this.leftMotors = leftMotors;
    this.invertL = invertL;
    leftLead = leftMotors[0];
    leftDrive = new MotorControllerGroup(leftMotors);
    leftDrive.setInverted(invertL);

    // Right motors.
    this.rightMotors = rightMotors;
    this.invertR = invertR;
    rightLead = rightMotors[0];
    rightDrive = new MotorControllerGroup(rightMotors);
    rightDrive.setInverted(invertR);

    diffDrive = new DifferentialDrive(leftDrive, rightDrive);
    driveOdometer = new DifferentialDriveOdometry(Rotation2d.fromDegrees(pigeon2.getRawAngles()[0]));

    deviceName = "Differential_Drivetrain";
    this.driveConfig = driveConfig;
    this.pigeon2 = pigeon2;
  }

  /**
   * Arcade driving controls (movement separated by axis). Slow mode is enabled.
   * 
   * @param netSpeed  Logitudinal (forward and backward) speed.
   * @param turnSpeed Lateral (left and right) speed.
   */
  public void arcadeDrive(double netSpeed, double turnSpeed) {
    arcadeDrive(netSpeed, turnSpeed, slowModeActive);
  }

  /**
   * Arcade driving controls (movement separated by axis). Slow mode can be
   * enabled or disabled.
   * 
   * @param netSpeed    Logitudinal (forward and backward) speed. -1 to 1.
   * @param turnSpeed   Lateral (left and right) speed. -1 to 1.
   * @param useSlowMode Enable or disable slow mode.
   */
  public void arcadeDrive(double netSpeed, double turnSpeed, boolean useSlowMode) {
    if (useSlowMode) {
      netSpeed *= driveConfig.getSlowModeForwardMultiplier();
      turnSpeed *= driveConfig.getSlowModeTurnMultiplier();
    }
    diffDrive.arcadeDrive(netSpeed, turnSpeed);
  }

  /**
   * Tank driving controls (movement separated by side of drivetrain). Slow mode
   * is enabled.
   * 
   * @param leftSpeed  Speed of left motors. -1 to 1.
   * @param rightSpeed Speed of right motors. -1 to 1.
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    tankDrive(leftSpeed, rightSpeed, slowModeActive);
  }

  /**
   * Tank driving controls (movement separated by side of drivetrain). Slow mode
   * can be enabled or disabled.
   * 
   * @param leftSpeed   Speed of left motors. -1 to 1.
   * @param rightSpeed  Speed of right motors. -1 to 1.
   * @param useSlowMode Enable or disable slow mode.
   */
  public void tankDrive(double leftSpeed, double rightSpeed, boolean useSlowMode) {
    if (useSlowMode) {
      leftSpeed *= driveConfig.getSlowModeForwardMultiplier();
      rightSpeed *= driveConfig.getSlowModeForwardMultiplier();
    }
    diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * Tank driving controls (movement separated by side of drivetrain) utilizing
   * voltage.
   * 
   * @param leftVoltage  Voltage for left motors. Negative to invert output.
   * @param rightVoltage Voltage for right motors. Negative to invert output.
   */
  public void tankDriveVoltage(double leftVoltage, double rightVoltage) {
    leftDrive.setVoltage(leftVoltage);
    rightDrive.setVoltage(rightVoltage);
    diffDrive.feed();
    System.out.println("LV: " + leftVoltage + " RV: " + rightVoltage);
  }

  // Both sides of drivetrain.

  /** Resets drivetrain encoders. */
  public void resetDriveEncoders() {
    leftLead.setSelectedSensorPosition(0);
    rightLead.setSelectedSensorPosition(0);
  }

  /**
   * Turns the Falcon 500's builtin brake mode on or off for the drivetrain.
   * 
   * @param isEnabled Enable or disable brake mode.
   */
  public void setDrivetrainBrakeMode(boolean isEnabled) {
    BreakerCTREUtil.setBrakeMode(isEnabled, leftMotors);
    BreakerCTREUtil.setBrakeMode(isEnabled, rightMotors);
  }

  // Left motors.

  /** Returns left motor ticks. */
  public double getLeftDriveTicks() {
    return invertL ? -leftLead.getSelectedSensorPosition() : leftLead.getSelectedSensorPosition();
  }

  /** Returns left motor ticks converted into inches. */
  public double getLeftDriveInches() {
    return (getLeftDriveTicks() / driveConfig.getTicksPerInch());
  }

  /** Returns left motor ticks converted into meters. */
  public double getLeftDriveMeters() {
    return Units.inchesToMeters(getLeftDriveInches());
  }

  /** Returns left motor velocity in raw sensor units (ticks per 100ms). */
  public double getLeftDriveVelocityRSU() {
    return invertL ? -leftLead.getSelectedSensorVelocity() : leftLead.getSelectedSensorVelocity();
  }

  /** Returns an instance of the drivetrain's left side lead motor */
  public WPI_TalonFX getLeftLeadMotor() {
    return leftLead;
  }

  // Right motors.

  /** Returns right motor ticks. */
  public double getRightDriveTicks() {
    return invertR ? -rightLead.getSelectedSensorPosition() : rightLead.getSelectedSensorPosition();
  }

  /** Returns right motor ticks converted into inches. */
  public double getRightDriveInches() {
    return getRightDriveTicks() / driveConfig.getTicksPerInch();
  }

  /** Returns right motor ticks converted into meters. */
  public double getRightDriveMeters() {
    return Units.inchesToMeters(getRightDriveInches());
  }

  /** Returns right motor velocity in raw sensor units (ticks per 100ms). */
  public double getRightDriveVelocityRSU() {
    return invertR ? -rightLead.getSelectedSensorVelocity() : rightLead.getSelectedSensorVelocity();
  }

  /** Returns an instance of the drivetrain's right side lead motor */
  public WPI_TalonFX getRightLeadMotor() {
    return rightLead;
  }

  // Controllers and kinematics.

  /**
   * Returns a SimpleMotorFeedforward controller with values specified by the
   * drivetrain config.
   */
  public SimpleMotorFeedforward getFeedforward() {
    return new SimpleMotorFeedforward(driveConfig.getFeedForwardKs(), driveConfig.getFeedForwardKv(),
        driveConfig.getFeedForwardKa());
  }

  /**
   * Returns a DifferentialDriveKinematics with values specified by the drivetrain
   * config.
   */
  public DifferentialDriveKinematics getKinematics() {
    return driveConfig.getKinematics();
  }

  /**
   * Returns the PIDController for the left drivetrain motors with values
   * specified by the drivetrain config.
   */
  public PIDController getLeftPIDController() {
    return driveConfig.getLeftPID();
  }

  /**
   * Returns the PIDController for the right drivetrain motors with values
   * specified by the drivetrain config.
   */
  public PIDController getRightPIDController() {
    return driveConfig.getRightPID();
  }

  /**
   * Returns DifferentialDriveWheelSpeeds with values specified by the drivetrain
   * config.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        BreakerUnits.inchesToMeters((getLeftDriveVelocityRSU() / driveConfig.getTicksPerInch()) * 10),
        BreakerUnits.inchesToMeters((getRightDriveVelocityRSU() / driveConfig.getTicksPerInch()) * 10));
  }

  public BreakerDiffDriveState getDiffDriveState() {
    return new BreakerDiffDriveState(getWheelSpeeds(), getLeftDriveMeters(), getRightDriveMeters());
  }

  private void calculateMovementState(double timeToLastUpdateMilisecods) {
    BreakerMovementState2d curMovementState = BreakerMath.movementStateFromChassisSpeedsAndPreviousState(
        getOdometryPoseMeters(), getFieldRelativeChassisSpeeds(), timeToLastUpdateMilisecods, prevMovementState);
    prevMovementState = curMovementState;
  }

  public WPI_TalonFX[] getLeftMotors() {
    return leftMotors;
  }

  public WPI_TalonFX[] getRightMotors() {
    return rightMotors;
  }

  @Override
  public void updateOdometry() {
    driveOdometer.update(Rotation2d.fromDegrees(pigeon2.getRawAngles()[0]), getLeftDriveMeters(),
        getRightDriveMeters());
    // calculateMovementState((Timer.getFPGATimestamp() -
    // prevOdometryUpdateTimestamp) * 1000);
    prevOdometryUpdateTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public Pose2d getOdometryPoseMeters() {
    return driveOdometer.getPoseMeters();
  }

  @Override
  public void runSelfTest() {
    faultStr = null;
    health = DeviceHealth.NOMINAL;

    StringBuilder work = new StringBuilder();
    for (WPI_TalonFX motorL : leftMotors) {
      Faults motorFaults = new Faults();
      motorL.getFaults(motorFaults);
      if (motorFaults.hasAnyFault()) {
        health = DeviceHealth.FAULT;
        work.append(" MOTOR ID (" + motorL.getDeviceID() + ") FAULTS: ");
        work.append(BreakerCTREUtil.getMotorFaultsAsString(motorFaults));
      }
    }
    for (WPI_TalonFX motorR : rightMotors) {
      Faults motorFaults = new Faults();
      motorR.getFaults(motorFaults);
      if (motorFaults.hasAnyFault()) {
        health = DeviceHealth.FAULT;
        work.append(" MOTOR ID (" + motorR.getDeviceID() + ") FAULTS: ");
        work.append(BreakerCTREUtil.getMotorFaultsAsString(motorFaults));
      }
    }
    faultStr = work.toString();
  }

  @Override
  public void setOdometryPosition(Pose2d newPose) {
    resetDriveEncoders();
    driveOdometer.resetPosition(newPose, Rotation2d.fromDegrees(pigeon2.getRawAngles()[0]));
  }

  @Override
  public Object getBaseOdometer() {
    return driveOdometer;
  }

  @Override
  public BreakerMovementState2d getMovementState() {
    return curMovementState;
  }

  @Override
  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return driveConfig.getKinematics().toChassisSpeeds(getWheelSpeeds());
  }

  @Override
  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return BreakerMath.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(), getOdometryPoseMeters().getRotation());
  }

  @Override
  public ChassisSpeeds getFieldRelativeChassisSpeeds(BreakerGenericOdometer odometer) {
    return BreakerMath.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(),
        odometer.getOdometryPoseMeters().getRotation());
  }

  @Override
  public boolean isUnderAutomaticControl() {
    return isAutoPowerManaged;
  }

  @Override
  public DevicePowerMode getPowerMode() {
    return powerMode;
  }

  @Override
  public DevicePowerMode managePower(BreakerPowerManagementConfig managementConfig) {

    return null;
  }

  @Override
  public void overrideAutomaticPowerManagement(DevicePowerMode manualPowerMode) {
    isAutoPowerManaged = false;

  }

  @Override
  public void returnToAutomaticPowerManagement() {
    isAutoPowerManaged = true;
  }

  @Override
  public void periodic() {
    updateOdometry();
  }
}
