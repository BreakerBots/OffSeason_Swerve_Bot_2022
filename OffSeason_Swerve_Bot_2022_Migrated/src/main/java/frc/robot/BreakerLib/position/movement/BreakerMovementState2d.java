// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.movement;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.physics.Breaker3AxisForces;

/** Represents an object's 2D (linear: XY / Angular: Y) position (m & rad), velocity (m & rad/s), acceleration(m & rad/s^2), and jerk (m & rad /s^3) at a given time. */
public class BreakerMovementState2d {
    private Pose2d position;
    private Breaker3AxisForces[] positionDerivatives;

    /**
     * Creates a new BreakerMovementState2d using a given position, velocity, acceleration, and jerk.
     * 
     * @param position
     * @param velocity
     * @param acceleration
     * @param jerk
     */
    public BreakerMovementState2d(Pose2d position, Breaker3AxisForces... derivitivesOfPosition) {
        this.position = position;
        this.positionDerivatives = derivitivesOfPosition;
    }

     /**
     * Creates a new BreakerMovementState2d and uses default values for the position, velocity, acceleration, and jerk.
     * 
     * @param position
     * @param velocity
     * @param acceleration
     */
    public BreakerMovementState2d() {
        position = new Pose2d();
        positionDerivatives = new Breaker3AxisForces[0];
    }

    public Pose2d estimateFuturePose(double deltaTimeSeconds) {
        double prevDirX = positionDerivatives[positionDerivatives.length - 1].getLinearForces().getMagnatudeX();
        double prevDirY = positionDerivatives[positionDerivatives.length - 1].getLinearForces().getMagnatudeY();
        double prevDirT = positionDerivatives[positionDerivatives.length - 1].getAngularForce();
        for (int i = positionDerivatives.length - 1; i >= 0; i++) {
            prevDirX = positionDerivatives[i].getLinearForces().getMagnatudeX() + (prevDirX * deltaTimeSeconds);
            prevDirY = positionDerivatives[i].getLinearForces().getMagnatudeY() + (prevDirY * deltaTimeSeconds);
            prevDirT = positionDerivatives[i].getAngularForce() + (prevDirT * deltaTimeSeconds);
        }
        double x = position.getX() + (prevDirX * deltaTimeSeconds);
        double y = position.getY() + (prevDirY * deltaTimeSeconds);
        double t = position.getRotation().getRadians() + (prevDirT * deltaTimeSeconds);
        return new Pose2d(x, y, new Rotation2d(t));
    }

    /**
     * @return The position component of this BreakerMovementState2d
     */
    public Pose2d getPositionComponent() {
        return position;
    }
 
    public Breaker3AxisForces[] getDerivativesOfPosition() {
        return Arrays.copyOf(positionDerivatives, positionDerivatives.length);
    }


    /** Follows indexes of derivative array, so 1st dirivitive would be at index 0 */
    public Breaker3AxisForces getDerivativefromIndex(int derivativeIndex) {
        if (derivativeIndex >= positionDerivatives.length || derivativeIndex < 0 || positionDerivatives[derivativeIndex] == null) {
            return new Breaker3AxisForces();
        }
        return positionDerivatives[derivativeIndex];
    }

    @Override
    public String toString() {
        return String.format("Breaker3AxisForces(Position: %s, Dirivitives: %s)", position, Arrays.toString(positionDerivatives));
    }
}
