// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.action.diffactionauto.waypoint;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** A path of positional waypoints q*/
public class BreakerWaypointPath {
    private TrapezoidProfile.Constraints constraints;
    private Translation2d[] waypoints;
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
}

    