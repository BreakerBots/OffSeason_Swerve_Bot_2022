// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.waypoint;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** A path of positional waypoints. */
public class BreakerWaypointPath {
    
    private final TrapezoidProfile.Constraints constraints;
    private final Translation2d[] waypoints;

    /**
     * Creates path of waypoints.
     * 
     * @param constraints Trapezoid profile constraints.
     * @param waypoints 
     */
    public BreakerWaypointPath(TrapezoidProfile.Constraints constraints, Translation2d... waypoints) {
        this.constraints = constraints;
        this.waypoints = waypoints;
    }
    
    /** @return Overall distance of path in meters. */
    public double getTotalPathDistance() {
        double dist = 0;
        for (int i = 1; i < waypoints.length; i++) {
            dist += waypoints[i-1].getDistance(waypoints[i]);
        }
        return dist;
    }

    public TrapezoidProfile.Constraints getConstraints() {
        return constraints;
    }

    public Translation2d[] getWaypoints() {
        return waypoints;
    }
    
    /**
     * Averages speed of two waypoint paths and 
     * 
     * @param other Path to concatenate to end of original path.
     * @return New waypoint path with averaged constraints and concatenated points.
     */
    public BreakerWaypointPath concatenate(BreakerWaypointPath other) {
        Translation2d[] newWaypoints = new Translation2d[getWaypoints().length + other.getWaypoints().length];
        for (int i = 0; i < waypoints.length; i++) {
            newWaypoints[i] = waypoints[i];
        }
        for (int i = waypoints.length; i < newWaypoints.length; i++) {
            newWaypoints[i] = other.getWaypoints()[i];
        }
        // Averages trapezoid profile constraints and concatenates autopath
        return new BreakerWaypointPath(
            new TrapezoidProfile.Constraints(
                (constraints.maxVelocity + other.constraints.maxVelocity) / 2.0, 
                (constraints.maxAcceleration + other.constraints.maxAcceleration) / 2.0),
            newWaypoints);
    }
}

    