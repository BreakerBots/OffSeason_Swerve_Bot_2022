// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.physics.vector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolable;

/**
 * Represents a point with 3 axial vectors of ajustable magnitudes (one on each
 * X, Y, and Z axis)
 */
public class BreakerVector3 implements BreakerInterpolable<BreakerVector3> {

    private double magnatudeX;
    private double magnatudeY;
    private double magnatudeZ;
    private double magnitude;
    private Rotation3d vectorRotation;

    /** Creates a BreakerVector3 based on given x, y, and z forces.
     * 
     * @param magnatudeX X-axis force relative to field.
     * @param magnatudeY Y-axis force relative to field.
     * @param magnatudeZ Z-axis force relative to field.
    */
    public BreakerVector3(double magnatudeX, double magnatudeY, double magnatudeZ) {
        this.magnatudeX = magnatudeX;
        this.magnatudeY = magnatudeY;
        this.magnatudeZ = magnatudeZ;
        magnitude = Math.sqrt(Math.pow(magnatudeX, 2) + Math.pow(magnatudeY, 2) + Math.pow(magnatudeZ, 2));
        vectorRotation = new Rotation3d(0.0, Math.atan2(magnatudeZ, (Math.sqrt(magnatudeX*magnatudeX+magnatudeY*magnatudeY))),
                Math.atan2(magnatudeY, magnatudeX)); // need to check this math
    }

    public BreakerVector3() {
        magnatudeX = 0;
        magnatudeY = 0;
        magnatudeZ = 0;
        magnitude = 0;
        vectorRotation = new Rotation3d();
    }

    /** Private constructor for BreakerVector3 with all the fixings. */
    private BreakerVector3(double magnatudeX, double magnatudeY, double magnatudeZ, double magnitude,
            Rotation3d vectorRotation) {
        this.magnatudeX = magnatudeX;
        this.magnatudeY = magnatudeY;
        this.magnatudeZ = magnatudeZ;
        this.magnitude = magnitude;
        this.vectorRotation = vectorRotation;
    }

    /**
     * Creates a BreakerVector3 from given magnitude and force rotation.
     * 
     * @param magnitude     Total force of the vector.
     * @param vectorRotation Rotation of forces.
     */
    public static BreakerVector3 fromMagnitudeAndvectorRotation(double magnitude, Rotation3d vectorRotation) {
        double x = magnitude * (Math.cos(vectorRotation.getZ()) * Math.cos(vectorRotation.getY()));
        double y = magnitude * (Math.sin(vectorRotation.getZ()) * Math.cos(vectorRotation.getY()));
        double z = magnitude * (Math.sin(vectorRotation.getY()));
        return new BreakerVector3(x, y, z, magnitude, vectorRotation);
    }

    /** converts an instance of WPILib's translation3d class into a vector.
     *  This exists because of the Tranlation2d classes suppport of various vector opperations */
    public static BreakerVector3 fromTranslation(Translation3d translationToVectorize) {
        return new BreakerVector3(translationToVectorize.getX(), translationToVectorize.getY(), translationToVectorize.getY());
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
     * @return double
     */
    public double getMagnatudeZ() {
        return magnatudeZ;
    }

    
    /** 
     * @return double
     */
    public double getMagnitude() {
        return magnitude;
    }

    
    /** 
     * @return Rotation3d
     */
    public Rotation3d getVectorRotation() {
        return vectorRotation;
    }

    
    /** 
     * @param rotation
     * @return BreakerVector3
     */
    public BreakerVector3 rotateBy(Rotation3d rotation) {
        return BreakerVector3.fromMagnitudeAndvectorRotation(magnitude, vectorRotation.plus(rotation));
    }

    
    /** 
     * @param outher
     * @return BreakerVector3
     */
    public BreakerVector3 minus(BreakerVector3 outher) {
        return new BreakerVector3(magnatudeX - outher.magnatudeX, magnatudeY - outher.magnatudeY, magnatudeZ - outher.magnatudeZ);
    }

    
    /** 
     * @param outher
     * @return BreakerVector3
     */
    public BreakerVector3 plus(BreakerVector3 outher) {
        return new BreakerVector3(magnatudeX + outher.magnatudeX, magnatudeY + outher.magnatudeY, magnatudeZ + outher.magnatudeZ);
    }

    
    /** 
     * @return BreakerVector3
     */
    public BreakerVector3 unaryMinus()  {
        return new BreakerVector3(-magnatudeX, -magnatudeY, -magnatudeZ);
    }

    
    /** 
     * @param scalar
     * @return BreakerVector3
     */
    public BreakerVector3 times(double scalar) {
        return new BreakerVector3(magnatudeX * scalar, magnatudeY * scalar, magnatudeZ * scalar);
    }

    
    /** 
     * @param scalar
     * @return BreakerVector3
     */
    public BreakerVector3 div(double scalar) {
        return new BreakerVector3(magnatudeX / scalar,  magnatudeY / scalar, magnatudeY / scalar);
    }
    
    
    /** 
     * @return Translation3d
     */
    public Translation3d getAsTranslation() {
        return new Translation3d(magnatudeX, magnatudeY, magnatudeZ);
    }

    
    /** 
     * @return BreakerVector3
     */
    public BreakerVector3 getUnitVector() {
        return BreakerVector3.fromMagnitudeAndvectorRotation(1, vectorRotation);
    }

    
    /** 
     * @param endValue
     * @param t
     * @return BreakerVector3
     */
    @Override
    public BreakerVector3 interpolate(BreakerVector3 endValue, double t) {
        double interX = MathUtil.interpolate(magnatudeX, endValue.getMagnatudeX(), t);
        double interY = MathUtil.interpolate(magnatudeY, endValue.getMagnatudeY(), t);
        double interZ = MathUtil.interpolate(magnatudeZ, endValue.getMagnatudeZ(), t);
        return new BreakerVector3(interX, interY, interZ);
    }
    
    /** [0] = X, [1] = Y, [2] = Z */
    @Override
    public double[] getInterpolatableData() {
        return new double[] { magnatudeX, magnatudeY, magnatudeZ };
    }

    
    /** 
     * @param interpolatableData
     * @return BreakerVector3
     */
    @Override
    public BreakerVector3 fromInterpolatableData(double[] interpolatableData) {
        return new BreakerVector3(interpolatableData[0], interpolatableData[1], interpolatableData[2]);
    }

    
    /** 
     * @return String
     */
    @Override
    public String toString() {
        return "Vector Magnatude:" + magnitude + ", X-Magnatude: " + magnatudeX + ", Y-Magnatude: " + magnatudeY
                + ", Z-Magnatude: " + magnatudeZ + ", Vector Angles: " + vectorRotation.toString();
    }
}
