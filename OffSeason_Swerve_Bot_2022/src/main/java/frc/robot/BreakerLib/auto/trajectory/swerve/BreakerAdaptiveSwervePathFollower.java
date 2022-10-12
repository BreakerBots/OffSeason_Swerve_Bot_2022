// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.auto.trajectory.swerve;

// import java.util.ArrayList;
// import java.util.List;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import frc.robot.BreakerLib.auto.trajectory.swerve.rotation.BreakerGenericSwerveRotationSupplier;
// import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolableDouble;
// import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolatableDoubleArray;
// import frc.robot.BreakerLib.util.math.interpolation.interpolateingmaps.BreakerInterpolatingTreeMap;
// import frc.robot.BreakerLib.util.math.interpolation.interpolateingmaps.BreakerLegrangeInterpolateingTreeMap;

// /** Add your docs here. */
// public class BreakerAdaptiveSwervePathFollower {
//     private BreakerLegrangeInterpolateingTreeMap<Double, BreakerInterpolatableDoubleArray> rawPoseInterpolMap;
//     private BreakerInterpolatingTreeMap<Double, BreakerInterpolableDouble> rotInterpolMap;
//     private BreakerGenericSwerveRotationSupplier rotSupplier;
//     private List<Translation2d> waypoints;
//     private List<Translation2d> transTargets;
//     private List<Pose2d> poseTargets;
//     private boolean usesPoseTargets, usesSuppliedRotation;
//     private int legrangeSubdivs;


//     public BreakerAdaptiveSwervePathFollower(int legrangeSubdivisions, BreakerGenericSwerveRotationSupplier rotationSupplier, Translation2d... targetPoints) {
//         rawPoseInterpolMap = new BreakerLegrangeInterpolateingTreeMap<>();
//         transTargets = new ArrayList<>();
//         for(Double i = 0.0; i < targetPoints.length; i++) {
//             rawPoseInterpolMap.put(i, new BreakerInterpolatableDoubleArray(targetPoints[i.intValue()].getX(), targetPoints[i.intValue()].getY()));
//         }
//         for(Translation2d tgt: targetPoints) {
//             transTargets.add(tgt);
//         }
//         waypoints = getWaypoints();
        
//     }

//     public BreakerAdaptiveSwervePathFollower(int legrangeSubdivisions, Pose2d... targetPoses) {
        
//     }

//     private List<Translation2d> getWaypoints() {
//         List<Translation2d> retWp = new ArrayList<>();
//         double delta = legrangeSubdivs == 0 ? 1.0 : 1.0 / legrangeSubdivs;
//         for(Double i = 0.0; i < rawPoseInterpolMap.size(); i += delta) {
//             BreakerInterpolatableDoubleArray dar = i == Math.ceil(i) || i == Math.floor(i) ? rawPoseInterpolMap.get(i) : rawPoseInterpolMap.getInterpolatedValue(i);
//             retWp.add(new Translation2d(dar.getValue()[0], dar.getValue()[1]));
//         }
//         return retWp;
//     }
// }
