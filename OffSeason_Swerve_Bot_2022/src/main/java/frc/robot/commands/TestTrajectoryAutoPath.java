// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
    ProfiledPIDController thetaPID =  new ProfiledPIDController(0.05, 0.0, 2.0, new TrapezoidProfile.Constraints(0.5, 0.5));
    thetaPID.enableContinuousInput(0, 2*Math.PI);
    BreakerSwerveAutoPathFollowerConfig swerveFollowerConfig = new BreakerSwerveAutoPathFollowerConfig(
        drivetrain.getBaseDrivetrain(),
        new HolonomicDriveController(new PIDController(2.0, 0.0, 0.1), new PIDController(2.0, 0.0, 0.1),
            thetaPID));

    BreakerTrajectoryPath traj1 = new BreakerTrajectoryPath(TrajectoryGenerator.generateTrajectory(
        new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d()),
        BreakerTrajectoryUtil.toTranslationWaypointList(
            new Translation2d(1.0, 0.0)
        ),
        new Pose2d(new Translation2d(2.0, 0.0), new Rotation2d()),
        new TrajectoryConfig(0.5, 0.5)), true);

    BreakerSwerveRotationSupplier rotSup = new BreakerSwerveRotationSupplier(new BreakerRotationPoint(new Rotation2d(), 0.0), new BreakerRotationPoint(new Rotation2d(Math.PI), 5.0));

    //BreakerSwerveRotationSupplier rotSup = new BreakerSwerveRotationSupplier((Double time) -> (new Rotation2d((time/2) * Math.PI)));
    addCommands(
        new BreakerStartTrajectoryPath(drivetrain.getBaseDrivetrain(),  new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d())),
        new BreakerSwerveAutoPathFollower(swerveFollowerConfig, rotSup, traj1));
  }
}
