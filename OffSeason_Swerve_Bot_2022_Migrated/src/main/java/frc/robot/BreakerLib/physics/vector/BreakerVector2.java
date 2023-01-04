// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.physics.vector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolable;

/**
 * represents a 2 dimentional vector, a uantity with bolth magnatude and direction. Here representd as a total magnatude, angular direction, 
 * x manatude component, and y magnatude component
 */
public class BreakerVector2 implements BreakerInterpolable<BreakerVector2> {
    private Rotation2d vectorRotation;
    private double magnatude;
    private double magnatudeX;
    private double magnatudeY;

    /**
     * creates a new BreakerVector2 from the magnatudes of the vector's X and Y
     * components
     */
    public BreakerVector2(double magnatudeX, double magnatudeY) {
        this.magnatudeX = magnatudeX;
        this.magnatudeY = magnatudeY;
        vectorRotation = new Rotation2d(Math.atan2(magnatudeY, magnatudeX));
        magnatude = Math.sqrt(Math.pow(magnatudeX, 2) + Math.pow(magnatudeY, 2));
    }

    private BreakerVector2(double magnatudeX, double magnatudeY, double magnatude, Rotation2d vectorRotation) {
        this.magnatudeX = magnatudeX;
        this.magnatudeY = magnatudeY;
        this.magnatude = magnatude;
        this.vectorRotation = vectorRotation;
    }

    /** creates an empty BreakerVector2 with 0's for all values */
    public BreakerVector2() {
     magnatudeX = 0;
        magnatudeY = 0;
        vectorRotation = Rotation2d.fromDegrees(0);
        magnatude = 0;
    }

    /**
     * creates a new BreakerVector2 from the vectors Magnatude and the vectors angle
     * in the Yaw angular axis
     */
    public static BreakerVector2 fromForceAndRotation(Rotation2d vectorRotation, double magnatude) {
        return new BreakerVector2(magnatude * Math.cos(vectorRotation.getRadians()),
                magnatude * Math.sin(vectorRotation.getRadians()), magnatude, vectorRotation);
    }

    /** converts an instance of WPILib's translation2d class into a vector.
     *  This exists because of the Tranlation2d classes suppport of various vector opperations */
    public static BreakerVector2 fromTranslation(Translation2d translationToVectorize) {
        return new BreakerVector2(translationToVectorize.getX(), translationToVectorize.getY());
    }

    
    /** 
     * @return double
     */
    public double getMagnatude() {
        return magnatude;
    }

    
    /** 
     * @return Rotation2d
     */
    public Rotation2d getVectorRotation() {
        return vectorRotation;
    }

    
    /** 
     * @return double
     */
    public double getMagnatudeX() {
        return magnatudeX;
    }

    
    /** 
     * @return double
     */
    public double getMagnatudeY() {
        return magnatudeY;
    }

    
    /** 
     * @param outher
     * @return BreakerVector2
     */
    public BreakerVector2 plus(BreakerVector2 outher) {
        return new BreakerVector2(magnatudeX + outher.magnatudeX, magnatudeY + outher.magnatudeY);
    }

    
    /** 
     * @param outher
     * @return BreakerVector2
     */
    public BreakerVector2 minus(BreakerVector2 outher) {
        return new BreakerVector2(magnatudeX - outher.magnatudeX, magnatudeY - outher.magnatudeY);
    }

    
    /** 
     * @return BreakerVector2
     */
    public BreakerVector2 unaryMinus()  {
        return new BreakerVector2(-magnatudeX, -magnatudeY);
    }

    
    /** 
     * @param scalar
     * @return BreakerVector2
     */
    public BreakerVector2 times(double scalar) {
        return new BreakerVector2(magnatudeX * scalar, magnatudeY * scalar);
    }

    
    /** 
     * @param scalar
     * @return BreakerVector2
     */
    public BreakerVector2 div(double scalar) {
        return new BreakerVector2(magnatudeX / scalar,  magnatudeY / scalar);
    }
    
    
    /** 
     * @return Translation2d
     */
    public Translation2d getAsTranslation() {
        return new Translation2d(magnatudeX, magnatudeY);
    }

    
    /** 
     * @param rotation
     * @return BreakerVector2
     */
    public BreakerVector2 rotateBy(Rotation2d rotation) {
        double cos = Math.cos(rotation.getRadians());
        double sin = Math.sin(rotation.getRadians());
        return new BreakerVector2((this.magnatudeX * cos) - (this.magnatudeY * sin), (this.magnatudeX * sin) + (this.magnatudeY * cos));
    }

    
    /** 
     * @return BreakerVector2
     */
    public BreakerVector2 getUnitVector() {
        return BreakerVector2.fromForceAndRotation(vectorRotation, 1.0);
    }

    
    /** 
     * @param obj
     * @return boolean
     */
    @Override
    public boolean equals(Object obj) {
        return (Math.abs(((BreakerVector2) obj).magnatudeX - magnatudeX) < 1E-9)
                && (Math.abs(((BreakerVector2) obj).magnatudeY - magnatudeY) < 1E-9);
    }

    
    /** 
     * @param endValue
     * @param t
     * @return BreakerVector2
     */
    @Override
    public BreakerVector2 interpolate(BreakerVector2 endValue, double t) {
        double interX = MathUtil.interpolate(magnatudeX, endValue.getMagnatudeX(), t);
        double interY = MathUtil.interpolate(magnatudeY, endValue.getMagnatudeY(), t);
        return new BreakerVector2(interX, interY);
    }

    /** [0] = X, [1] = Y */
    @Override
    public double[] getInterpolatableData() {
        return new double[] { magnatudeX, magnatudeY };
    }

    
    /** 
     * @param interpolatableData
     * @return BreakerVector2
     */
    @Override
    public BreakerVector2 fromInterpolatableData(double[] interpolatableData) {
        return new BreakerVector2(interpolatableData[0], interpolatableData[1]);
    }

    
    /** 
     * @return String
     */
    @Override
    public String toString() {
       return String.format("BreakerVector2(Vector_Magnatude: %.2f, X-Magnatude: %.2f, Y-Magnatude: %.2f,  Vector_Angle: %s)", magnatude, magnatudeX, magnatudeY, vectorRotation.toString());
    }

}
