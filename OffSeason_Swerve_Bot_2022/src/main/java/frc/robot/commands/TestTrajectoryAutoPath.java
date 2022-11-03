// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BreakerLib.auto.trajectory.BreakerTrajectoryUtil;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerStartTrajectoryPath;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerTrajectoryPath;
import frc.robot.BreakerLib.auto.trajectory.swerve.BreakerFollowSwerveTrajectory;
import frc.robot.BreakerLib.auto.trajectory.swerve.BreakerFollowSwerveTrajectoryConfig;
import frc.robot.BreakerLib.auto.trajectory.swerve.rotation.BreakerRotationPoint;
import frc.robot.BreakerLib.auto.trajectory.swerve.rotation.BreakerSwerveRotationSupplier;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestTrajectoryAutoPath extends SequentialCommandGroup {
  /** Creates a new TestAutoPath. */
  public TestTrajectoryAutoPath(Drive drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    BreakerTrajectoryPath traj = new BreakerTrajectoryPath(TrajectoryGenerator.generateTrajectory(new Pose2d(0.0,0.0, drivetrain.getBaseDrivetrain().getOdometryPoseMeters().getRotation()), BreakerTrajectoryUtil.toTranslationWaypointList(new Translation2d(0,1), new Translation2d(1, 1), new Translation2d(2, 0)), new Pose2d(3,0, new Rotation2d()), new TrajectoryConfig(drivetrain.getBaseDrivetrain().getConfig().getMaxForwardVel(), 2.0));
    addCommands(
      new BreakerStartTrajectoryPath(drivetrain.getBaseDrivetrain(), new Pose2d(0.0,0.0, drivetrain.getBaseDrivetrain().getOdometryPoseMeters().getRotation())),
      new BreakerFollowSwerveTrajectory(new BreakerFollowSwerveTrajectoryConfig(drivetrain.getBaseDrivetrain(), new PIDController(1.0, 0.0, 0.0), new PIDController(1.0, 0.0, 0.0), 0.1, 0.0, 0.0, 0.0, drivetrain.getBaseDrivetrain().getConfig().getMaxAngleVel()), new BreakerSwerveRotationSupplier(new BreakerRotationPoint(Rotation2d.fromDegrees(45), 1.0), new BreakerRotationPoint(Rotation2d.fromDegrees(-45), 2.0)), true, drivetrain.getBaseDrivetrain(), traj)
    );
  }
}
