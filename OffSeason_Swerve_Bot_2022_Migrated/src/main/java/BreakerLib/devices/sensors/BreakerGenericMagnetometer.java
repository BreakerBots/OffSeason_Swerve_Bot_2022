// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package BreakerLib.devices.sensors;

/** Interface for 3-axis magnitometers with digital compass features  */
public interface BreakerGenericMagnetometer {
    /** X[0], Y[1], Z[2], in microteslas */
   public abstract double[] getRawFieldStrengths();

   /** X[0], Y[1], Z[2], in microteslas. */
   public abstract double[] getBiasedFieldStrengths();

   /** @return Magnetic field strength in microteslas. */
   public abstract double getCompassFieldStrength();

   /** @return Angular heading of the compass in +-180 degrees. */
   public abstract double getCompassHeading();

   /** @return Raw angular heading of the compass in degrees. */
   public abstract double getRawCompassHeading();

}
