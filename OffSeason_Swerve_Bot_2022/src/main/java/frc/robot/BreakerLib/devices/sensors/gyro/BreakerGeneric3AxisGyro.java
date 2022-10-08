// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.position.geometry.BreakerRotation3d;

/** FRC 3-axis gyroscope interface. */
public interface BreakerGeneric3AxisGyro extends BreakerGenericGyro {

  /** @return Pitch angle within +-180 degrees. */
  public abstract double getPitchDegrees();

  /** @return Roll angle within +-180 degrees. */
  public abstract double getRollDegrees();

  /** @return Pitch angle as {@link Rotation2d} within +-180 degrees. */
  public abstract Rotation2d getPitchRotation2d();

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

  public abstract double getRawPitch();

  public abstract double getRawRoll();

  /** Sets desired angle to given angle.
   * 
   * @param value Angle value in degrees.
   * @param angleNum 0 = pitch, 1 = yaw, 2 = roll. Defaults to yaw. 
   */
  public default void set(double value, int angleNum) {
    switch (angleNum) {
      case 0:
        setPitch(value);
        break;
      case 2:
        setRoll(value);
        break;
      default:
        setYaw(value);
        break;
    }
  }


  /** Sets pitch to angle value. */
  public abstract void setPitch(double value);

  /** Sets roll to angle value. */
  public abstract void setRoll(double value);

  /** Returns angular velocities in degrees per sec.
   *  x=0, y=1, z=2.
   */
  public abstract double[] getRawGyroRates();

  public abstract double getRawPitchRate();

  public abstract double getRawRollRate();

  /** @return Angular velocity pitch.(deg/sec) */
  public abstract double getPitchRate();

  /** @return Angular velocity roll.(deg/sec) */
  public abstract double getRollRate();

  /** @return Pitch, yaw, and roll as {@link BreakerRotation3d} */
  public abstract BreakerRotation3d getRawRotation3d();
}
