package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.util.ShootingCalculator;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final SuperstructureIO io;
  private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

  private final LoggedTunableNumber defaultShootSpeed =
      new LoggedTunableNumber(
          "Superstructure/Shooter/ManualShootSpeed",
          SuperstructureConstants.Shooter.DEFAULT_SHOOT_SPEED);
  private final LoggedTunableNumber defaultKickerSpeed =
      new LoggedTunableNumber(
          "Superstructure/Kicker/ManualKickSpeed",
          SuperstructureConstants.Kicker.DEFAULT_VELOCITY.in(RotationsPerSecond));

  public Superstructure(SuperstructureIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure", inputs);
    LoggedTracer.record("Superstructure");
  }

  public void startConveyorAndKicker() {
    io.setKickerRPS(RotationsPerSecond.of(defaultKickerSpeed.get()));
    io.setConveyorRPS(SuperstructureConstants.Conveyor.DEFAULT_VELOCITY);
  }

  public void startShooter(AngularVelocity velocity) {
    io.setShooterRPS(velocity);
  }

  public void stopAll() {
    io.setConveyorRPS(RotationsPerSecond.of(0));
    io.stopKicker();
    io.stopShooter();
  }

  public boolean isAtSpeed(AngularVelocity velocity) {
    return io.isAtSpeed(velocity);
  }

  public Command tuneSpeed() {
    return runOnce(
        () -> {
          io.setShooterRPS(RotationsPerSecond.of(defaultShootSpeed.get()));
          io.setKickerRPS((RotationsPerSecond.of(defaultKickerSpeed.get())));
          io.setConveyorRPS(SuperstructureConstants.Conveyor.DEFAULT_VELOCITY);
        });
  }

  public Command shootAtSpeed(AngularVelocity speed) {
    return runOnce(
        () -> {
          io.setShooterRPS(speed);
          io.setKickerRPS(SuperstructureConstants.Kicker.DEFAULT_VELOCITY);
          io.setConveyorRPS(SuperstructureConstants.Conveyor.DEFAULT_VELOCITY);
        });
  }

  public Command shootAtHub() {
    return run(
        () -> {
          var calc = ShootingCalculator.getInstance();
          var target = calc.getHub();
          var delta = calc.update(target);
          io.setShooterRPS(ShootingCalculator.rpsForDistance(delta.distance()));
          io.setKickerRPS(SuperstructureConstants.Kicker.DEFAULT_VELOCITY);
          io.setConveyorRPS(SuperstructureConstants.Conveyor.DEFAULT_VELOCITY);
        });
  }

  public Command stop() {
    return runOnce(
        () -> {
          io.setConveyorRPS(RotationsPerSecond.of(0));
          io.stopKicker();
          io.stopShooter();
        });
  }
}
