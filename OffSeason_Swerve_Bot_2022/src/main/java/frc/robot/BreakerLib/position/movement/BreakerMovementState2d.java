// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.movement;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.physics.Breaker3AxisForces;
import frc.robot.BreakerLib.physics.BreakerVector2;

/** Represents an objects 2 dimentional (linear: XY / Angular: Y) position (meters & rad), velocity (m & rad/s), acceleration( m & rad/s/s), and jerk (m & rad /s/s/s) at a given time */
public class BreakerMovementState2d {
    Pose2d position;
    Breaker3AxisForces velocity;
    Breaker3AxisForces acceleration;
    Breaker3AxisForces jerk;

    /**
     * Creates a new BreakerMovementState2d using a given position, velocity, acceleration, and jerk.
     * 
     * @param position
     * @param velocity
     * @param acceleration
     * @param jerk
     */
    public BreakerMovementState2d(Pose2d position, Breaker3AxisForces velocity, Breaker3AxisForces acceleration, Breaker3AxisForces jerk) {
        this.position = position;
        this.acceleration = acceleration;
        this.jerk = jerk;
    }

    /**
     * Creates a new BreakerMovementState2d using a given position, velocity and acceleration.
     * 
     * @param position
     * @param velocity
     * @param acceleration
     */
    public BreakerMovementState2d(Pose2d position, Breaker3AxisForces velocity, Breaker3AxisForces acceleration) {
        this.position = position;
        this.acceleration = acceleration;
        jerk = new Breaker3AxisForces();
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
        velocity = new Breaker3AxisForces();
        acceleration = new Breaker3AxisForces();
        jerk = new Breaker3AxisForces();
    }

    /**
     * @return The position component of this BreakerMovementState2d
     */
    public Pose2d getPositionComponent() {
        return position;
    }

    /**
     * @return The acceleration component of this BreakerMovementState2d.
     */
    public Breaker3AxisForces getAccelerationComponent() {
        return acceleration;
    }
    /**
     * @return The velocity component of this BreakerMovementState2d.
     */
    public Breaker3AxisForces getVelocityComponent() {
        return velocity;
    }

    /**
     * @return The jerk component of this BreakerMovementState2d.
     */
    public Breaker3AxisForces getJerkCompoenet() {
        return jerk;
    }
}
