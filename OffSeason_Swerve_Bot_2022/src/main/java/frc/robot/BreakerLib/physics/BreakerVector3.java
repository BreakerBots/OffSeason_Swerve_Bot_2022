// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.physics;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.position.geometry.BreakerRotation3d;
import frc.robot.BreakerLib.util.math.BreakerMath;
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
    private BreakerRotation3d vectorRotation;

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
        vectorRotation = new BreakerRotation3d(new Rotation2d(Math.atan2(magnatudeZ, (Math.sqrt(magnatudeX*magnatudeX+magnatudeY*magnatudeY)))),
                new Rotation2d(Math.atan2(magnatudeY, magnatudeX))); // need to check this math
    }

    public BreakerVector3() {
        magnatudeX = 0;
        magnatudeY = 0;
        magnatudeZ = 0;
        magnitude = 0;
        vectorRotation = new BreakerRotation3d();
    }

    /** Private constructor for BreakerVector3 with all the fixings. */
    private BreakerVector3(double magnatudeX, double magnatudeY, double magnatudeZ, double magnitude,
            BreakerRotation3d vectorRotation) {
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
    public static BreakerVector3 fromMagnitudeAndvectorRotation(double magnitude, BreakerRotation3d vectorRotation) {
        double x = magnitude * (Math.cos(vectorRotation.getYaw().getRadians()) * Math.cos(vectorRotation.getPitch().getRadians()));
        double y = magnitude * (Math.sin(vectorRotation.getYaw().getRadians()) * Math.cos(vectorRotation.getPitch().getRadians()));
        double z = magnitude * (Math.sin(vectorRotation.getPitch().getRadians()));
        return new BreakerVector3(x, y, z, magnitude, vectorRotation);
    }

    public double getMagnatudeX() {
        return magnatudeX;
    }

    public double getMagnatudeY() {
        return magnatudeY;
    }

    public double getMagnatudeZ() {
        return magnatudeZ;
    }

    public double getMagnitude() {
        return magnitude;
    }

    public BreakerRotation3d getVectorRotation() {
        return vectorRotation;
    }

    public BreakerVector3 rotate(BreakerRotation3d rotation) {
        return BreakerVector3.fromMagnitudeAndvectorRotation(magnitude, vectorRotation.plus(rotation));
    }

    public BreakerVector3 minus(BreakerVector3 outher) {
        return new BreakerVector3(magnatudeX - outher.magnatudeX, magnatudeY - outher.magnatudeY, magnatudeZ - outher.magnatudeZ);
    }

    public BreakerVector3 plus(BreakerVector3 outher) {
        return new BreakerVector3(magnatudeX + outher.magnatudeX, magnatudeY + outher.magnatudeY, magnatudeZ + outher.magnatudeZ);
    }

    @Override
    public BreakerVector3 interpolate(double interpolendValue, double highKey, BreakerVector3 highVal,
            double lowKey, BreakerVector3 lowVal) {
        double interX = BreakerMath.interpolateLinear(interpolendValue, lowKey, highKey, lowVal.getMagnatudeX(),
                highVal.getMagnatudeX());
        double interY = BreakerMath.interpolateLinear(interpolendValue, lowKey, highKey, lowVal.getMagnatudeY(),
                highVal.getMagnatudeZ());
        double interZ = BreakerMath.interpolateLinear(interpolendValue, lowKey, highKey, lowVal.getMagnatudeZ(),
                highVal.getMagnatudeZ());

        return new BreakerVector3(interX, interY, interZ);
    }

    @Override
    public BreakerVector3 getSelf() {
        return this;
    }

    /** [0] = X, [1] = Y, [2] = Z */
    @Override
    public double[] getInterpolatableData() {
        return new double[] { magnatudeX, magnatudeY, magnatudeZ };
    }

    @Override
    public BreakerVector3 fromInterpolatableData(double[] interpolatableData) {
        return new BreakerVector3(interpolatableData[0], interpolatableData[1], interpolatableData[2]);
    }

    @Override
    public String toString() {
        return "Vector Magnatude:" + magnitude + ", X-Magnatude: " + magnatudeX + ", Y-Magnatude: " + magnatudeY
                + ", Z-Magnatude: " + magnatudeZ + ", Vector Angles: " + vectorRotation.toString();
    }
}
