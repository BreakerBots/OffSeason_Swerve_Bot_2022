// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;

/** Interface for all BreakerLib classes capable of generteing odometry data */
public interface BreakerGenericOdometer {

  /**
   * Set the odometer's current position.
   * 
   * @param newPose Pose to set the odometer to.
   */
  public abstract void setOdometryPosition(Pose2d newPose);

  /**
   * Changes odometer translation without affecting rotation.
   * 
   * @param newTranslation New odometer translation.
   */
  public default void setOdometryTranslation(Translation2d newTranslation) {
    setOdometryPosition(new Pose2d(newTranslation, getOdometryPoseMeters().getRotation()));
  }

  /**
   * Changes odometer rotation without affecting translation.
   * 
   * @param newRotation New odometer rotation.
   */
  public default void setOdometryRotation(Rotation2d newRotation) {
    setOdometryPosition(new Pose2d(getOdometryPoseMeters().getTranslation(), newRotation));
  }

  /** Resets odometer translation and rotation. */
  public default void resetOdometryPosition() {
    setOdometryPosition(new Pose2d());
  }

  /** Resets odometer translation to 0. */
  public default void resetOdometryTranslation() {
    setOdometryTranslation(new Translation2d());
  }

  /** Resets odometer rotation to 0. */
  public default void resetOdometryRotation() {
    setOdometryRotation(new Rotation2d());
  }

  /** @return Odometery pose in meters. */
  public abstract Pose2d getOdometryPoseMeters();

  /** @return Movement state of object. */
  public abstract BreakerMovementState2d getMovementState();

  /** @return Chassis speeds relative to robot. */
  public abstract ChassisSpeeds getRobotRelativeChassisSpeeds();

  /** @return Chassis speeds relative to field. */
  public abstract ChassisSpeeds getFieldRelativeChassisSpeeds();
}
