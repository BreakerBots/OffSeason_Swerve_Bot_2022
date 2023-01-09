// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.waypoint;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** A path of positional waypoints q*/
public class BreakerWaypointPath {
    private final TrapezoidProfile.Constraints constraints;
    private final Translation2d[] waypoints;
    public BreakerWaypointPath(TrapezoidProfile.Constraints constraints, Translation2d... waypoints) {
        this.constraints = constraints;
        this.waypoints = waypoints;
    }
    
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

    public BreakerWaypointPath concatinate(BreakerWaypointPath outher) {
        Translation2d[] newWaypoints = new Translation2d[getWaypoints().length + outher.getWaypoints().length];
        for (int i = 0; i < newWaypoints.length; i++) {
            if (i < waypoints.length) {
                newWaypoints[i] = waypoints[i];
            } else {
                newWaypoints[i] = outher.getWaypoints()[i - (newWaypoints.length - 1)];
            }
        }
        return new BreakerWaypointPath(
            new TrapezoidProfile.Constraints(
                (constraints.maxVelocity + outher.constraints.maxVelocity) / 2.0, 
                (constraints.maxAcceleration + outher.constraints.maxAcceleration) / 2.0),
            newWaypoints);
    }
}

    