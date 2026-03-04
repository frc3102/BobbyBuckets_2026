package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.LoggedTracer;
import frc.robot.util.ShootingCalculator;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final SuperstructureIO io;
  private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

  public Superstructure(SuperstructureIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure", inputs);
    LoggedTracer.record("Superstructure");
  }

  private AngularVelocity rpsForDistance(Distance distance) {
    var feet = distance.in(Feet);
    if (feet <= 5) {
      return RotationsPerSecond.of(40);
    } else if (feet <= 7) {
      return RotationsPerSecond.of(50);
    } else if (feet <= 10) {
      return RotationsPerSecond.of(60);
    } else {
      return RotationsPerSecond.of(70);
    }
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
          io.setShooterRPS(rpsForDistance(delta.distance()));
          io.setKickerRPS(SuperstructureConstants.Kicker.DEFAULT_VELOCITY);
          io.setConveyorRPS(SuperstructureConstants.Conveyor.DEFAULT_VELOCITY);
        });
  }

  public Command stop() {
    return runOnce(
        () -> {
          io.setConveyorRPS(RotationsPerSecond.of(0));
          io.setKickerRPS(RotationsPerSecond.of(0));
          io.setShooterRPS(RotationsPerSecond.of(0));
        });
  }
}
