package frc.robot.subsystems.loader;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTracer;
import frc.robot.subsystems.intake.feed.IntakeFeedConstants;
import org.littletonrobotics.junction.Logger;

public class Loader extends SubsystemBase {
  private final LoaderIO io;
  private final LoaderIOInputsAutoLogged inputs = new LoaderIOInputsAutoLogged();

  private final SysIdRoutine sysIdRoutine;

  public Loader(LoaderIO io) {
    this.io = io;

    this.sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Second), // override default ramp rate (1 V/s)
                Volts.of(2.0), // override default step voltage (7 V)
                null, // use default timeout (10 s)
                state -> SignalLogger.writeString("SysId_State", state.toString())),
            new SysIdRoutine.Mechanism(output -> io.setVoltage(output), null, this));
    SysIdRoutineChooser.getInstance().addOption("Loader Voltage", sysIdRoutine);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Loader", inputs);
    LoggedTracer.record("Loader");
  }

  public Command startLoader() {
    return runOnce(
        () -> {
          io.setVoltage(IntakeFeedConstants.VOLTAGE);
        });
  }

  public Command stopLoader() {
    return runOnce(
        () -> {
          io.setVoltage(Volts.of(0));
        });
  }
}
