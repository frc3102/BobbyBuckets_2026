// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team6328.util.LoggedTracer;
import frc.robot.commands.DriveCommands;
import frc.robot.game.GameState;
import frc.robot.game.GameStateIO;
import frc.robot.game.GameStateIORobot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.controls.Haptics;
import frc.robot.subsystems.controls.HapticsIO;
import frc.robot.subsystems.controls.HapticsIOXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.intake.feed.IntakeFeed;
import frc.robot.subsystems.intake.feed.IntakeFeedIO;
import frc.robot.subsystems.intake.feed.IntakeFeedIOTalonFX;
import frc.robot.subsystems.intake.tilt.IntakeTilt;
import frc.robot.subsystems.intake.tilt.IntakeTiltIO;
import frc.robot.subsystems.intake.tilt.IntakeTiltIOTalonFX;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureIO;
import frc.robot.subsystems.superstructure.SuperstructureIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.ShootingCalculator;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final IntakeFeed intakeFeed;
  private final IntakeTilt intakeTilt;
  private final Superstructure superstructure;
  private final Haptics haptics;
  private final Elevator elevator;

  private final GameState gameState;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandGenericHID coDriverController = new CommandGenericHID(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));
        intakeFeed = new IntakeFeed(new IntakeFeedIOTalonFX());
        intakeTilt = new IntakeTilt(new IntakeTiltIOTalonFX());
        superstructure = new Superstructure(new SuperstructureIOTalonFX());
        gameState = new GameState(new GameStateIORobot());
        haptics = new Haptics(new HapticsIOXboxController(driverController));
        elevator = new Elevator(new ElevatorIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose));
        intakeFeed = new IntakeFeed(new IntakeFeedIOTalonFX());
        intakeTilt = new IntakeTilt(new IntakeTiltIOTalonFX());
        superstructure = new Superstructure(new SuperstructureIOTalonFX());
        gameState = new GameState(new GameStateIORobot());
        haptics = new Haptics(new HapticsIOXboxController(driverController));
        elevator = new Elevator(new ElevatorIOTalonFX());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        intakeFeed = new IntakeFeed(new IntakeFeedIO() {});
        intakeTilt = new IntakeTilt(new IntakeTiltIO() {});
        superstructure = new Superstructure(new SuperstructureIO() {});
        gameState = new GameState(new GameStateIO() {});
        haptics = new Haptics(new HapticsIO() {});
        elevator = new Elevator(new ElevatorIO() {});

        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    ShootingCalculator.init(drive::getPose, drive::getChassisSpeeds, gameState);
    // Configure the button bindings
    configureButtonBindings();
  }

  public void periodic() {
    gameState.periodic();
    LoggedTracer.record("RobotContainer");
  }

  public void disabledPeriodic() {
    gameState.periodic();
    LoggedTracer.record("DisabledRobotContainer");
  }

  private Rotation2d getAngleToHub() {
    var calc = ShootingCalculator.getInstance();
    var target = calc.getHub();
    var delta = calc.update(target);
    return new Rotation2d(delta.angle());
  }

  private Rotation2d getAngleToTrench() {
    var calc = ShootingCalculator.getInstance();
    var target = calc.getNearestTrench();
    var delta = calc.update(target);
    return new Rotation2d(delta.angle());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    driverController.a().onTrue(intakeFeed.startIntake());
    driverController.b().onTrue(intakeFeed.stopIntake());
    driverController.x().onTrue(intakeTilt.extendHopper());
    driverController.y().onTrue(intakeTilt.retractHopper());
    driverController
        .rightTrigger()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                this::getAngleToHub));
    driverController
        .leftTrigger()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                this::getAngleToTrench));
    driverController.back().onTrue(drive.zeroGyroscope());
    driverController.povDown().onTrue(elevator.goToPosition(ElevatorConstants.BOTTOM_POSITION));
    driverController.povUp().onTrue(elevator.goToPosition(ElevatorConstants.TOP_POSITION));

    coDriverController
        .button(10)
        .onTrue(superstructure.shootAtHub())
        .onFalse(superstructure.stop());
    // coDriverController.button(11).onTrue(new StopShooter(loader, launcher));

    // coDriverController.button(7).onTrue(launcher.startAtVoltage(Volts.of(6))).onFalse(launcher.stopLauncher());
    // coDriverController.button(8).onTrue(launcher.stopLauncher());
    // coDriverController.button(4).onTrue(loader.startLoader());
    // coDriverController.button(5).onTrue(loader.stopLoader());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
