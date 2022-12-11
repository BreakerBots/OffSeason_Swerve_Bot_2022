// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package BreakerLib.auto.waypoint.pathfinder;

import java.util.ArrayList;

import BreakerLib.auto.waypoint.BreakerWaypointPath;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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

    public BreakerWaypointPath getAsWaypointPath(TrapezoidProfile.Constraints constraints, Translation2d startPoint, Translation2d endPoint) {
        Translation2d[] waypoints = new Translation2d[pathNodes.size()];
        for (int i = 1; i < waypoints.length - 1; i++) {
            BreakerPathfinderNode node = pathNodes.get(i);
            double x = ((double)(node.getGridPosX()) + 0.5) * nodeSideLength;
            double y = ((double)(node.getGridPosY()) + 0.5) * nodeSideLength;

            waypoints[i] = new Translation2d(x, y);
        }
        waypoints[0] = startPoint;
        waypoints[waypoints.length-1] = endPoint;
        return new BreakerWaypointPath(constraints, waypoints);
    }
}
