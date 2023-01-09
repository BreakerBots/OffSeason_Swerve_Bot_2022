// Prototyping a new example RobotContainer setup. 

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.BreakerLib.driverstation.gamepad.components.BreakerGamepadAnalogDeadbandConfig;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerTeleopSwerveDriveController;
import frc.robot.BreakerLib.util.robot.BreakerRobotConfig;
import frc.robot.BreakerLib.util.robot.BreakerRobotManager;
import frc.robot.BreakerLib.util.robot.BreakerRobotStartConfig;
import frc.robot.subsystems.Drive;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final BreakerXboxController controllerSys = new BreakerXboxController(0);
  private final BreakerPigeon2 imuSys = new BreakerPigeon2(5);
  private final Drive drivetrainSys = new Drive(imuSys);
  private final BreakerTeleopSwerveDriveController manualDriveCommand = new BreakerTeleopSwerveDriveController(
      drivetrainSys, controllerSys);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    robotManagerSetup();

    controllerSys.configDeadbands(new BreakerGamepadAnalogDeadbandConfig(0.06, 0.06, 0.06, 0.06));

    drivetrainSys.resetOdometryPosition();

    configureButtonBindings();
    drivetrainSys.setDefaultCommand(manualDriveCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    controllerSys.getButtonB().onTrue(new InstantCommand(drivetrainSys::toggleSlowMode));
    controllerSys.getButtonX()
        .onTrue(new InstantCommand(drivetrainSys::resetOdometryRotation));
  }

  private void robotManagerSetup() {
    BreakerRobotConfig robotConfig = new BreakerRobotConfig(new BreakerRobotStartConfig(5104, "BreakerBots",
        "Breaker Swerve", 202, "v1", "Yousif Alkhalaf, Roman Abrahamson"));
    
    BreakerRobotManager.setup(drivetrainSys, robotConfig);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return drivetrainSys.getTestSuite().stressTest(4.0, 4.0, 16.0);
  }
}
