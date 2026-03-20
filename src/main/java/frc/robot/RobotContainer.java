// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team6328.util.LoggedTracer;
import frc.robot.commands.AutoDistanceShoot;
import frc.robot.commands.AutoJiggleHopper;
import frc.robot.commands.AutoTiltAndWait;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.JiggleHopper;
import frc.robot.commands.ShootCommand;
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
import frc.robot.subsystems.intake.tilt.IntakeTiltConstants;
import frc.robot.subsystems.intake.tilt.IntakeTiltIO;
import frc.robot.subsystems.intake.tilt.IntakeTiltIOTalonFX;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
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

  private boolean headBackWarning = false;
  private boolean startShootingWarning = false;

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
    ShootingCalculator.init(drive::getPose, drive::getChassisSpeeds, gameState);
    registerNamedCommands();
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    if (Constants.TUNING_MODE) {
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
    }
    // Configure the button bindings
    configureButtonBindings();
  }

  public void periodic() {
    gameState.periodic();
    if (gameState.isRealMatch()) {
      // if we're in the head-back time and we haven't issued the warning, start the buzz
      if (gameState.isHeadBackWarning()) {
        if (!headBackWarning) {
          CommandScheduler.getInstance().schedule(haptics.headBackWarningCommand());
        }
        headBackWarning = true;
      } else {
        headBackWarning = false;
      }
      // if we're in the start-shooting time and we haven't issued the warning, start the buzz
      if (gameState.isGreenLightPreShift()) {
        if (!startShootingWarning) {
          CommandScheduler.getInstance().schedule(haptics.startShootingCommand());
          startShootingWarning = true;
        }
      } else {
        startShootingWarning = false;
      }
    }
    LoggedTracer.record("RobotContainer");
  }

  public void disabledPeriodic() {
    gameState.periodic();
    LoggedTracer.record("DisabledRobotContainer");
  }

  private static Rotation2d getAngleToHub() {
    var calc = ShootingCalculator.getInstance();
    var target = calc.getHub();
    var delta = calc.update(target);
    return new Rotation2d(delta.angle());
  }

  private static Rotation2d getAngleToTrench() {
    var calc = ShootingCalculator.getInstance();
    var target = calc.getNearestTrench();
    var delta = calc.update(target);
    return new Rotation2d(delta.angle());
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand("AutoShoot", new AutoDistanceShoot(superstructure));
    NamedCommands.registerCommand(
        "JiggleHopper",
        new AutoJiggleHopper(
            intakeTilt, IntakeTiltConstants.OUT_POSITION, IntakeTiltConstants.FEED_POSITION, 1.5));
    NamedCommands.registerCommand(
        "ShootAtHub", new ShootCommand(superstructure, RotationsPerSecond.of(45)));
    NamedCommands.registerCommand(
        "ShootAtHub50", new ShootCommand(superstructure, RotationsPerSecond.of(50)));
    // NamedCommands.registerCommand("ShootAtHub", superstructure.shootAtHub());
    NamedCommands.registerCommand("StopShooter", superstructure.stop());

    var aimCommand =
        DriveCommands.joystickDriveAtAngle(drive, () -> 0, () -> 0, RobotContainer::getAngleToHub)
            .until(
                () -> {
                  var angle = RobotContainer.getAngleToHub();
                  return drive.getRotation().getMeasure().isNear(angle.getMeasure(), 0.05);
                }).finallyDo((b) -> {
                  drive.stop();
                });

    NamedCommands.registerCommand("AimAtHub", aimCommand);
    NamedCommands.registerCommand(
        "ElevatorUp", elevator.goToPositionCommand(ElevatorConstants.TOP_POSITION));
    NamedCommands.registerCommand(
        "ElevatorClimb", new ClimbCommand(elevator, ElevatorConstants.CLIMB_POSITION));
    NamedCommands.registerCommand(
        "ElevatorDown", elevator.goToPositionCommand(ElevatorConstants.BOTTOM_POSITION));
    NamedCommands.registerCommand(
        "TiltOutAndWait", new AutoTiltAndWait(intakeTilt, IntakeTiltConstants.OUT_POSITION));
    NamedCommands.registerCommand("TiltOut", intakeTilt.extendHopper());
    NamedCommands.registerCommand("TiltIn", intakeTilt.retractHopper());
    NamedCommands.registerCommand("IntakeStart", intakeFeed.startIntake());
    NamedCommands.registerCommand("IntakeStop", intakeFeed.stopIntake());
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
    // driverController.rightBumper().onTrue(intakeFeed.startIntakeVoltage(Volts.of(5.5)));
    driverController
        .leftBumper()
        .whileTrue(
            new JiggleHopper(
                intakeTilt, IntakeTiltConstants.OUT_POSITION, IntakeTiltConstants.FEED_POSITION));

    driverController
        .rightTrigger()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                RobotContainer::getAngleToHub));
    driverController
        .leftTrigger()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                RobotContainer::getAngleToTrench));
    driverController.back().onTrue(drive.zeroGyroscope());
    driverController
        .povDown()
        .onTrue(elevator.goToPositionCommand(ElevatorConstants.BOTTOM_POSITION));
    driverController.povUp().onTrue(elevator.goToPositionCommand(ElevatorConstants.TOP_POSITION));
    driverController
        .povRight()
        .onTrue(new ClimbCommand(elevator, ElevatorConstants.CLIMB_POSITION));
    driverController
        .povLeft()
        .onTrue(Commands.runOnce(() -> elevator.setVoltage(Volts.of(-1)), elevator))
        .onFalse(
            Commands.runOnce(
                    () -> {
                      elevator.setVoltage(Volts.of(0));
                    },
                    elevator)
                .andThen(elevator.zeroElevator()));
    coDriverController.button(10).whileTrue(new AutoDistanceShoot(superstructure));
    coDriverController
        .button(1)
        .whileTrue(
            new ShootCommand(
                superstructure,
                RotationsPerSecond.of(SuperstructureConstants.Shooter.TRENCH_SHOOT_SPEED)));

    // coDriverController.button(12).whileTrue(new TuningShot(superstructure));
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
