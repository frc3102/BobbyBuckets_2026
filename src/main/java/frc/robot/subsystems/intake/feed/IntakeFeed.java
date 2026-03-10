package frc.robot.subsystems.intake.feed;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTracer;
import org.littletonrobotics.junction.Logger;

public class IntakeFeed extends SubsystemBase {

  private final IntakeFeedIO io;
  private final IntakeFeedIOInputsAutoLogged inputs = new IntakeFeedIOInputsAutoLogged();

  private final SysIdRoutine sysIdRoutine;

  public IntakeFeed(IntakeFeedIO io) {
    this.io = io;
    this.sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Second), // override default ramp rate (1 V/s)
                Volts.of(2.0), // override default step voltage (7 V)
                null, // use default timeout (10 s)
                state -> SignalLogger.writeString("SysId_State", state.toString())),
            new SysIdRoutine.Mechanism(output -> io.setVoltage(output), null, this));
    SysIdRoutineChooser.getInstance().addOption("Intake Feed Voltage", sysIdRoutine);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeFeed", inputs);
    LoggedTracer.record("Intake");
  }

  public Command startIntakeVoltage(Voltage volts) {
    return runOnce(
        () -> {
          io.setVoltage(volts);
        });
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
          io.setVoltage(Volts.of(0));
        });
  }
}
