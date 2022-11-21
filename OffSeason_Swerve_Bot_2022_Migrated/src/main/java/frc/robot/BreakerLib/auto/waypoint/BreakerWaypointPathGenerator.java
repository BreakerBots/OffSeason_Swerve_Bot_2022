// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.waypoint;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.BreakerLib.auto.waypoint.pathfinder.BreakerPathfinder;
import frc.robot.BreakerLib.auto.waypoint.pathfinder.BreakerPathfinderNodeGrid;
import frc.robot.BreakerLib.auto.waypoint.pathfinder.BreakerPathfinderPath;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolatableDoubleArray;
import frc.robot.BreakerLib.util.math.interpolation.interpolateingmaps.BreakerLegrangeInterpolateingTreeMap;

/** Add your docs here. */
public class BreakerWaypointPathGenerator {
    public static BreakerWaypointPath generateWaypointPath(double maxVelocity, double maxAcceleration, double interpolationResolution, Translation2d... waypoints) {
        BreakerLegrangeInterpolateingTreeMap<Double, BreakerInterpolatableDoubleArray>  interMap = new BreakerLegrangeInterpolateingTreeMap<>();
        ArrayList<Translation2d> newWaypoints = new ArrayList<>();
        double dt = 1.0/interpolationResolution;
        for (int i = 0; i < waypoints.length; i++) {
            Translation2d wp = waypoints[i];
            interMap.put((double) i, new BreakerInterpolatableDoubleArray(new double[]{wp.getX(), wp.getY()}));
        }

        for (double i = 0; i < waypoints.length; i+=dt) {
            double[] interArr = interMap.getInterpolatedValue(i).getValue();
            newWaypoints.add(new Translation2d(interArr[0], interArr[1]));
        }

        return new BreakerWaypointPath(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration), newWaypoints.toArray(new Translation2d[newWaypoints.size()]));
    }

    public static BreakerWaypointPath findWaypointPath(double pathSearchTimeoutSeconds, double maxVelocity, double maxAcceleration, Translation2d startPoint, Translation2d endPoint, BreakerPathfinderNodeGrid nodeGrid) throws Exception {
        try {
            BreakerPathfinder pathfinder = new BreakerPathfinder(maxAcceleration, nodeGrid.getInstance(nodeGrid.getNodeFromPosition(startPoint), nodeGrid.getNodeFromPosition(startPoint)));
            BreakerPathfinderPath pfPath = pathfinder.calculatePath();
            return pfPath.getAsWaypointPath(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        } catch (Exception e) {
            throw e;
        }
    }

    public static BreakerWaypointPath findWaypointPath(double pathSearchTimeoutSeconds, double maxVelocity, double maxAcceleration, BreakerPathfinderNodeGrid nodeGrid, Translation2d... pathPoints) throws Exception {
        BreakerWaypointPath[] wpPaths = new BreakerWaypointPath[pathPoints.length - 1];
        double sTimeout = pathSearchTimeoutSeconds / (pathPoints.length - 1);
        for (int i = 0; i < pathPoints.length - 1; i++) {
            wpPaths[i] = findWaypointPath(sTimeout, maxVelocity, maxAcceleration, pathPoints[i], pathPoints[i+1], nodeGrid);
        }
        BreakerWaypointPath path = wpPaths[0];
        for (int i = 1; i < wpPaths.length; i++) {
            path.concatinate(wpPaths[i]);
        }
        return path;
    }
}
