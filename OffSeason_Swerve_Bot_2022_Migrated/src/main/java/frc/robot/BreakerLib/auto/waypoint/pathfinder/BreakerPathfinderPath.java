// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.waypoint.pathfinder;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.BreakerLib.auto.waypoint.BreakerWaypointPath;

/** Add your docs here. */
public class BreakerPathfinderPath {
    private ArrayList<BreakerPathfinderNode> pathNodes;
    private double nodeSideLength;
    public BreakerPathfinderPath(double nodeSideLength, ArrayList<BreakerPathfinderNode> pathNodes) {
        this.pathNodes = new ArrayList<BreakerPathfinderNode>(pathNodes);
        this.nodeSideLength = nodeSideLength;
    }

    public ArrayList<BreakerPathfinderNode> getPathNodes() {
        return new ArrayList<BreakerPathfinderNode>(pathNodes);
    }

    public BreakerWaypointPath getAsWaypointPath(TrapezoidProfile.Constraints constraints) {
        Translation2d[] waypoints = new Translation2d[pathNodes.size()];
        for (int i = 0; i < waypoints.length; i++) {
            BreakerPathfinderNode node = pathNodes.get(i);
            double x = ((double)(node.getGridPosX()) + 0.5) * nodeSideLength;
            double y = ((double)(node.getGridPosY()) + 0.5) * nodeSideLength;

            waypoints[i] = new Translation2d(x, y);
        }
        return new BreakerWaypointPath(constraints, waypoints);
    }
}
