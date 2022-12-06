// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BreakerLib.auto.trajectory.BreakerTrajectoryUtil;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerStartTrajectoryPath;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerTrajectoryPath;
import frc.robot.BreakerLib.auto.trajectory.swerve.standard.BreakerSwerveAutoPathFollower;
import frc.robot.BreakerLib.auto.trajectory.swerve.standard.BreakerSwerveAutoPathFollowerConfig;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestTrajectoryAutoPath extends SequentialCommandGroup {
  
  /** Creates a new TestAutoPath. */
  public TestTrajectoryAutoPath(Drive drivetrain) {
    
    BreakerSwerveAutoPathFollowerConfig swerveFollowerConfig = new BreakerSwerveAutoPathFollowerConfig(drivetrain.getBaseDrivetrain(), 
        new HolonomicDriveController(new PIDController(2.0, 0.0, 0.1), new PIDController(2.0, 0.0, 0.1), new ProfiledPIDController(0.000000001, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0))));

    BreakerTrajectoryPath traj1 = new BreakerTrajectoryPath(TrajectoryGenerator.generateTrajectory(
        BreakerTrajectoryUtil.toPoseWaypointList(
          new Pose2d(), 
          new Pose2d(1.0, 0.0, new Rotation2d()),
          new Pose2d(1.0, -1.0, new Rotation2d()
          )),
        new TrajectoryConfig(0.5, 0.5)), true);
    
        
    addCommands(
      new BreakerStartTrajectoryPath(drivetrain.getBaseDrivetrain(), new Pose2d()),
      new BreakerSwerveAutoPathFollower(swerveFollowerConfig, traj1)
    );
  }
}
