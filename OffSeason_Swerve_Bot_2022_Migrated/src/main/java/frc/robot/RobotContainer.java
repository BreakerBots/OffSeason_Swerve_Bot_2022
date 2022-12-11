// Prototyping a new example RobotContainer setup. 

package frc.robot;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import BreakerLib.devices.sensors.imu.BreakerGenericIMU;
import BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import BreakerLib.driverstation.gamepad.components.BreakerGamepadAnalogDeadbandConfig;
import BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveConfig;
import BreakerLib.subsystem.cores.drivetrain.swerve.BreakerTeleopSwerveDriveController;
import BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerMK4iFalconSwerveModule;
import BreakerLib.util.BreakerArbitraryFeedforwardProvider;
import BreakerLib.util.robot.BreakerRobotConfig;
import BreakerLib.util.robot.BreakerRobotManager;
import BreakerLib.util.robot.BreakerRobotStartConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

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
  private final BreakerSwerveDrive drivetrainSys = swerveDriveSetup(imuSys);
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
    manualDriveCommand.addSlewRateLimiters(new SlewRateLimiter(2.0), new SlewRateLimiter(2.0),
        new SlewRateLimiter(4.0));
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
    controllerSys.getButtonB().toggleOnTrue(new InstantCommand(drivetrainSys::toggleSlowMode));
    controllerSys.getButtonX()
        .toggleOnTrue(new InstantCommand(drivetrainSys::resetOdometryRotation));
  }

  private BreakerSwerveDrive swerveDriveSetup(BreakerGenericIMU imu) {

    var driveFL = new WPI_TalonFX(FL_WHEEL_ID);
    var turnFL = new WPI_TalonFX(FL_ROTATION_ID);
    var encoderFL = new WPI_CANCoder(FL_ENCODER_ID);

    var driveFR = new WPI_TalonFX(FR_WHEEL_ID);
    var turnFR = new WPI_TalonFX(FR_ROTATION_ID);
    var encoderFR = new WPI_CANCoder(FR_ENCODER_ID);

    var driveBL = new WPI_TalonFX(BL_WHEEL_ID);
    var turnBL = new WPI_TalonFX(BL_ROTATION_ID);
    var encoderBL = new WPI_CANCoder(BL_ENCODER_ID);

    var driveBR = new WPI_TalonFX(BR_WHEEL_ID);
    var turnBR = new WPI_TalonFX(BR_ROTATION_ID);
    var encoderBR = new WPI_CANCoder(BR_ENCODER_ID);

    BreakerSwerveDriveConfig config = new BreakerSwerveDriveConfig(
        4.1148, 4.1148, 16.1148,
        1.25, 0.0, 0.05,
        0.35, 0.0, 0.0, 0.0,
        8.14, 4.0, 0.001, 4.1148,
        new BreakerArbitraryFeedforwardProvider(2.75, 0.2),
        FL_TRANSLATION, FR_TRANSLATION, BL_TRANSLATION, BR_TRANSLATION);
    config.setSlowModeMultipliers(0.5, 0.5);

    var frontLeftModule = new BreakerMK4iFalconSwerveModule(driveFL, turnFL, encoderFL, config, 121, true, true);
    frontLeftModule.setDeviceName(" FL_Module ");

    var frontRightModule = new BreakerMK4iFalconSwerveModule(driveFR, turnFR, encoderFR, config, -61, false, true);
    frontRightModule.setDeviceName(" FR_Module ");

    var backLeftModule = new BreakerMK4iFalconSwerveModule(driveBL, turnBL, encoderBL, config, 30.0, true, true);
    backLeftModule.setDeviceName(" BL_Module ");

    var backRightModule = new BreakerMK4iFalconSwerveModule(driveBR, turnBR, encoderBR, config, -176.0, false, true);
    backRightModule.setDeviceName(" BR_Module ");

    return new BreakerSwerveDrive(config, imu, frontLeftModule, frontRightModule, backLeftModule, backRightModule);
  }

  private void robotManagerSetup() {
    BreakerRobotConfig robotConfig = new BreakerRobotConfig(new BreakerRobotStartConfig(5104, "BreakerBots",
        "Breaker Swerve", 2022, "v1", "Yousif Alkhalaf, Roman Abrahamson"));
    
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
