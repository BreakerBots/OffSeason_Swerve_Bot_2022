// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import com.revrobotics.CANSparkMaxLowLevel.FollowConfig.Config;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BreakerLib.auto.trajectory.BreakerTrajectoryUtil;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerStartTrajectoryPath;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerTrajectoryPath;
import frc.robot.BreakerLib.auto.trajectory.swerve.BreakerSwerveAutoPathFollower;
import frc.robot.BreakerLib.auto.trajectory.swerve.BreakerSwerveAutoPathFollowerConfig;
import frc.robot.BreakerLib.auto.trajectory.swerve.rotation.BreakerRotationPoint;
import frc.robot.BreakerLib.auto.trajectory.swerve.rotation.BreakerSwerveRotationSupplier;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestTrajectoryAutoPath extends SequentialCommandGroup {
  /** Creates a new TestAutoPath. */
  public TestTrajectoryAutoPath(Drive drivetrain) {
    
    BreakerSwerveAutoPathFollowerConfig swerveFollowerConfig = new BreakerSwerveAutoPathFollowerConfig(drivetrain.getBaseDrivetrain(), 
        new HolonomicDriveController(new PIDController(kp, ki, kd), new PIDController(kp, ki, kd), new ProfiledPIDController(Kp, Ki, Kd, new TrapezoidProfile.Constraints(maxVel, maxAccel))), new Pose2d(x, y, rotation));

    BreakerTrajectoryPath traj1 = new BreakerTrajectoryPath(TrajectoryGenerator.generateTrajectory(
        BreakerTrajectoryUtil.toPoseWaypointList(waypoints),
        new TrajectoryConfig(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq)), true);
    
    addCommands(
      new BreakerStartTrajectoryPath(drivetrain.getBaseDrivetrain(), new Pose2d(0.0, 0.0, drivetrain.getBaseDrivetrain().getOdometryPoseMeters().getRotation())),
      new BreakerSwerveAutoPathFollower(swerveFollowerConfig, traj1)
    );
  }
}
