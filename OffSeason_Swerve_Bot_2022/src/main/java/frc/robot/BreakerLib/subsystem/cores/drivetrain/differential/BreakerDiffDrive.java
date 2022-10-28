// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.differential;

import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.position.odometry.differential.BreakerDiffDriveState;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;

/** Add your docs here. */
public abstract class BreakerDiffDrive extends BreakerGenericDrivetrain {
    private MotorControllerGroup leftDrive;
  
    private MotorControllerGroup rightDrive;
  
    private DifferentialDrive diffDrive;
    private BreakerDiffDriveConfig driveConfig;

    private DoubleSupplier leftTickSupplier, leftRPMSupplier;
    private DoubleSupplier rightTickSupplier, rightRPMSupplier;
  
    private BreakerGenericGyro imu;
    private DifferentialDriveOdometry driveOdometer;
    private BreakerMovementState2d prevMovementState = new BreakerMovementState2d();
    private BreakerMovementState2d curMovementState = new BreakerMovementState2d();
    private double prevOdometryUpdateTimestamp = 0;
  
    private boolean invertL;
    private boolean invertR;
  
    private boolean isAutoPowerManaged = true;
    private DevicePowerMode powerMode = DevicePowerMode.FULL_POWER_MODE;

    /** Base infastracture class from which all differential drivetrain types must inherit from. {@link BreakerFalconDiffDrive} {@link BreakerNeoDiffDrive}
     */
    protected BreakerDiffDrive(MotorController[] leftMotors, DoubleSupplier leftTickSupplier, DoubleSupplier leftRPMSupplier, boolean invertL,
        MotorController[] rightMotors, DoubleSupplier rightTickSupplier, DoubleSupplier rightRPMSupplier, boolean invertR,
        BreakerGenericGyro imu, BreakerDiffDriveConfig driveConfig) {

        this.invertL = invertL;
        this.invertR = invertR;

        leftDrive = new MotorControllerGroup(leftMotors);
        leftDrive.setInverted(invertL);
        this.leftTickSupplier = leftTickSupplier;
        this.leftRPMSupplier = leftRPMSupplier;

        rightDrive = new MotorControllerGroup(rightMotors);
        rightDrive.setInverted(invertR);
        this.rightTickSupplier = rightTickSupplier;
        this.rightRPMSupplier = rightRPMSupplier;
        
        diffDrive = new DifferentialDrive(leftDrive, rightDrive);

        driveOdometer = new DifferentialDriveOdometry(Rotation2d.fromDegrees(imu.getRawYaw()));

        deviceName = "Differential_Drivetrain";
        this.driveConfig = driveConfig;
        this.imu = imu;
     
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
  }

  // Both sides of drivetrain.

  /** Resets drivetrain encoders. */
  public abstract void resetDriveEncoders();

  /**
   * Turns the Falcon 500's builtin brake mode on or off for the drivetrain.
   * 
   * @param isEnabled Enable or disable brake mode.
   */
  public abstract void setDrivetrainBrakeMode(boolean isEnabled);

  // Left motors.

  /** Returns left motor ticks. */
  public double getLeftDriveTicks() {
    return invertL ? -leftTickSupplier.getAsDouble() : leftTickSupplier.getAsDouble();
  }

  /** Returns left motor ticks converted into inches. */
  public double getLeftDriveInches() {
    return (getLeftDriveTicks() / driveConfig.getTicksPerInch());
  }

  /** Returns left motor ticks converted into meters. */
  public double getLeftDriveMeters() {
    return Units.inchesToMeters(getLeftDriveInches());
  }

  /** Returns left motor velocity in raw sensor units. */
  public double getLeftDriveVelocityRPM() {
    return invertL ? -leftRPMSupplier.getAsDouble() : leftRPMSupplier.getAsDouble() ;
  }


  // Right motors.

  /** Returns right motor ticks. */
  public double getRightDriveTicks() {
    return invertR ? -rightTickSupplier.getAsDouble() : rightTickSupplier.getAsDouble();
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
  public double getRightDriveVelocityRPM() {
    return invertR ? -rightRPMSupplier.getAsDouble() : rightRPMSupplier.getAsDouble();
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
        Units.inchesToMeters(((getLeftDriveVelocityRPM() * driveConfig.getEncoderTicks()) / driveConfig.getTicksPerInch()) / 60),
        Units.inchesToMeters(((getRightDriveVelocityRPM() * driveConfig.getEncoderTicks()) / driveConfig.getTicksPerInch()) / 60)
    );
  }

  public BreakerDiffDriveState getDiffDriveState() {
    return new BreakerDiffDriveState(getWheelSpeeds(), getLeftDriveMeters(), getRightDriveMeters());
  }

  private void calculateMovementState(double timeToLastUpdateMilisecods) {
    BreakerMovementState2d curMovementState = BreakerMath.movementStateFromChassisSpeedsAndPreviousState(
        getOdometryPoseMeters(), getFieldRelativeChassisSpeeds(), timeToLastUpdateMilisecods, prevMovementState);
    prevMovementState = curMovementState;
  }

  @Override
  public void updateOdometry() {
    driveOdometer.update(Rotation2d.fromDegrees(imu.getRawYaw()), getLeftDriveMeters(),
        getRightDriveMeters());
    calculateMovementState((Timer.getFPGATimestamp() -
    prevOdometryUpdateTimestamp) * 1000);
    prevOdometryUpdateTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public Pose2d getOdometryPoseMeters() {
    return driveOdometer.getPoseMeters();
  }


  @Override
  public void setOdometryPosition(Pose2d newPose) {
    resetDriveEncoders();
    driveOdometer.resetPosition(newPose, Rotation2d.fromDegrees(imu.getRawYaw()));
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
