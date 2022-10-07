// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors.IMU;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.position.geometry.BreakerRotation3d;

/** FRC gyroscope interface. */
public interface BreakerGenericGyro {
  /** @return Pitch angle within +-180 degrees. */
  public abstract double getPitchDegrees();

  /** @return Yaw angle within +-180 degrees. */
  public abstract double getYawDegrees();

  /** @return Roll angle within +-180 degrees. */
  public abstract double getRollDegrees();

  /** @return Pitch angle as {@link Rotation2d} within +-180 degrees. */
  public abstract Rotation2d getPitchRotation2d();

  /** @return Yaw angle as {@link Rotation2d} within +-180 degrees. */
  public abstract Rotation2d getYawRotation2d();

  /** @return Roll angle as {@link Rotation2d} within +-180 degrees. */
  public abstract Rotation2d getRollRotation2d();

  /** Pitch, yaw, and roll as Rotation3d, all within +- 180 degrees. */
  public abstract BreakerRotation3d getRotation3d();

  /**
   * Returns raw yaw, pitch, and roll angles in an array.
   * <p>
   * yaw = 0, pitch = 1, roll = 2.
   */
  public abstract double[] getRawAngles();

  /** Resets yaw to 0 degrees */
  public abstract void reset();

  /** Sets yaw to given angle. */
  public abstract void set(double angle);

  /** Returns angular velocitys in degrees per sec */
  public abstract double[] getRawGyroRates();

  /** @return Angular velocity pitch.(deg/sec) */
  public abstract double getPitchRate();

  /** @return Angular velocity yaw.(deg/sec) */
  public abstract double getYawRate();

  /** @return Angular velocity roll.(deg/sec) */
  public abstract double getRollRate();

  /** @return Pitch, yaw, and roll as {@link BreakerRotation3d} */
  public abstract BreakerRotation3d getRawRotation3d();
}
