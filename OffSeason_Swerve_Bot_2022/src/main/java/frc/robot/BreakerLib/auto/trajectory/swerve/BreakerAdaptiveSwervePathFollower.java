// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.auto.trajectory.swerve;

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
    
//     public BreakerAdaptiveSwervePathFollower(int legrangeSubdivisions, BreakerGenericSwerveRotationSupplier rotationSupplier, Translation2d... targetPoints) {
//         rawPoseInterpolMap = new BreakerLegrangeInterpolateingTreeMap<>();
//         for(Double i = 0.0; i < targetPoints.length; i++) {
//             rawPoseInterpolMap.put(i, new BreakerInterpolatableDoubleArray(targetPoints[i.intValue()].getX(), targetPoints[i.intValue()].getY()));
//         }
        
        
//     }

//     public BreakerAdaptiveSwervePathFollower(int legrangeSubdivisions, Pose2d... targetPoses) {

//     }
// }
