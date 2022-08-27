// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int FL_WHEEL_ID = 10;
    public static final int FR_WHEEL_ID = 12;
    public static final int BL_WHEEL_ID = 14;
    public static final int BR_WHEEL_ID = 16;

    public static final int FL_ROTATION_ID = 11;
    public static final int FR_ROTATION_ID = 13;
    public static final int BL_ROTATION_ID = 15;
    public static final int BR_ROTATION_ID = 17;

    public static final int FL_ENCODER_ID = 20;
    public static final int FR_ENCODER_ID = 21;
    public static final int BL_ENCODER_ID = 22;
    public static final int BR_ENCODER_ID = 23;

    // FL is pos/pos, FR is pos/neg, BL is neg/pos, BR is neg/neg
    public static final Translation2d FL_TRANSLATION = new Translation2d(0.187325, 0.187325);
    public static final Translation2d FR_TRANSLATION = new Translation2d(0.187325, -0.187325);
    public static final Translation2d BL_TRANSLATION = new Translation2d(-0.187325, 0.187325);
    public static final Translation2d BR_TRANSLATION = new Translation2d(-0.187325, -0.187325);

    public static final int IMU_ID = 5;
}
