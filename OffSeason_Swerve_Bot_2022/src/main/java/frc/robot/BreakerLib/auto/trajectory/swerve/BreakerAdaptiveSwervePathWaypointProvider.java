// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.swerve;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.BreakerLib.auto.trajectory.swerve.rotation.BreakerGenericSwerveRotationSupplier;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolableDouble;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolatableDoubleArray;
import frc.robot.BreakerLib.util.math.interpolation.interpolateingmaps.BreakerInterpolatingTreeMap;
import frc.robot.BreakerLib.util.math.interpolation.interpolateingmaps.BreakerLegrangeInterpolateingTreeMap;

/** Add your docs here. */
public class BreakerAdaptiveSwervePathWaypointProvider {
    private BreakerLegrangeInterpolateingTreeMap<Double, BreakerInterpolatableDoubleArray> rawPoseInterpolMap;
    private BreakerGenericSwerveRotationSupplier rotSupplier;
    private List<Translation2d> waypoints;
    private List<Translation2d> transTargets;
    private int legrangeSubdivs;


    public BreakerAdaptiveSwervePathWaypointProvider(int legrangeSubdivisions, BreakerGenericSwerveRotationSupplier rotationSupplier, Translation2d... targetPoints) {
        rawPoseInterpolMap = new BreakerLegrangeInterpolateingTreeMap<>();
        transTargets = new ArrayList<>();
        rotSupplier = rotationSupplier;
        setNewTargetPoints(targetPoints);
    }

    public List<Translation2d> getCurrentTargetPoints() {
        return transTargets;
    }

    public List<Translation2d> getCurrentWaypoints() {
        return waypoints;
    }

    public Rotation2d getTargetRotation(double curTime) {
        rotSupplier.setCurrentTime(curTime);
        return rotSupplier.getRotation();
    }

    public void setNewTargetPoints(Translation2d... newTargets) {
        rawPoseInterpolMap.clear();
        transTargets.clear();
        for(Double i = 0.0; i < newTargets.length; i++) {
            rawPoseInterpolMap.put(i, new BreakerInterpolatableDoubleArray(newTargets[i.intValue()].getX(), newTargets[i.intValue()].getY()));
        }
        for(Translation2d tgt: newTargets) {
            transTargets.add(tgt);
        }
        waypoints = calculateWaypoints();
    }

    private List<Translation2d> calculateWaypoints() {
        List<Translation2d> retWp = new ArrayList<>();
        double delta = legrangeSubdivs == 0 ? 1.0 : 1.0 / legrangeSubdivs;
        for(Double i = 0.0; i < rawPoseInterpolMap.size(); i += delta) {
            BreakerInterpolatableDoubleArray dar = i == Math.ceil(i) || i == Math.floor(i) ? rawPoseInterpolMap.get(i) : rawPoseInterpolMap.getInterpolatedValue(i);
            retWp.add(new Translation2d(dar.getValue()[0], dar.getValue()[1]));
        }
        return retWp;
    }
}
