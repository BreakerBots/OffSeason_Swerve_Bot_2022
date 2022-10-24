// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.BreakerLib.driverstation.BreakerXboxController;
import frc.robot.BreakerLib.driverstation.BreakerXboxControllerDeadbandConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveController;
import frc.robot.BreakerLib.util.robotmanager.BreakerRobotConfig;
import frc.robot.BreakerLib.util.robotmanager.BreakerRobotManager;
import frc.robot.BreakerLib.util.robotmanager.BreakerRobotStartConfig;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final BreakerXboxController controllerSys = new BreakerXboxController(0);
  private final BreakerPigeon2 imuSys = new BreakerPigeon2(5);
  private final Drive drivetrainSys = new Drive(imuSys);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    
    BreakerRobotManager.setup(
        drivetrainSys.getBaseDrivetrain(),
        new BreakerRobotConfig(
            new BreakerRobotStartConfig(5104, "BreakerBots", "Offseason SwerveBot", 2022, "V1.5",
                "Roman Abrahamson, and Yousif Alkhalaf")));

    controllerSys.configAnalogInputDeadbands(new BreakerXboxControllerDeadbandConfig(0.01, 0.01, 0.01, 0.01));

    drivetrainSys.getBaseDrivetrain().resetOdometryPosition();;

    configureButtonBindings();
    drivetrainSys.getBaseDrivetrain()
        .setDefaultCommand(new BreakerSwerveDriveController(drivetrainSys.getBaseDrivetrain(), controllerSys));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    controllerSys.getdPadRight().whenPressed(new InstantCommand(drivetrainSys.getBaseDrivetrain()::toggleSlowMode));
    controllerSys.getButtonB().whenPressed(new InstantCommand(() -> {drivetrainSys.getBaseDrivetrain().setOdometryRotation(new Rotation2d());}));
    controllerSys.getButtonX().whenPressed(new InstantCommand(drivetrainSys.getBaseDrivetrain()::toggleSlowMode));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
