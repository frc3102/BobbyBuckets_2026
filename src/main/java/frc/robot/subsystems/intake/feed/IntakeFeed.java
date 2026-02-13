package frc.robot.subsystems.intake.feed;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeFeed extends SubsystemBase {

  private final IntakeFeedIO io;
  private final IntakeFeedIOInputsAutoLogged inputs = new IntakeFeedIOInputsAutoLogged();

  public IntakeFeed(IntakeFeedIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeFeed", inputs);
  }

  public Command startIntake() {
    return runOnce(
        () -> {
          io.setVoltage(IntakeFeedConstants.VOLTAGE);
        });
  }

  public Command stopIntake() {
    return runOnce(
        () -> {
          io.setVoltage(0.0);
        });
  }
}
