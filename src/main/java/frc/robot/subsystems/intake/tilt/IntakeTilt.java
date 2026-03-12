package frc.robot.subsystems.intake.tilt;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTracer;
import org.littletonrobotics.junction.Logger;

public class IntakeTilt extends SubsystemBase {
  private final IntakeTiltIO io;
  private final IntakeTiltIOInputsAutoLogged inputs = new IntakeTiltIOInputsAutoLogged();
  private final SysIdRoutine sysIdRoutine;

  public IntakeTilt(IntakeTiltIO io) {
    this.io = io;
    this.sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Second), // override default ramp rate (1 V/s)
                Volts.of(2.0), // override default step voltage (7 V)
                null, // use default timeout (10 s)
                state -> SignalLogger.writeString("SysId_State", state.toString())),
            new SysIdRoutine.Mechanism(output -> io.setVoltage(output), null, this));
    SysIdRoutineChooser.getInstance().addOption("Intake Tilt Voltage", sysIdRoutine);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeTilt", inputs);
    LoggedTracer.record("Tilt");
  }

  public void setPosition(double pos) {
    io.setAngle(Rotations.of(pos));
  }

  public boolean inPosition(double pos) {
    return inputs.position.isNear(Rotations.of(pos), 0.05);
  }

  public Command extendHopper() {
    return runOnce(
        () -> {
          io.setAngle(Rotations.of(IntakeTiltConstants.OUT_POSITION));
        });
  }

  public Command retractHopper() {
    return runOnce(
        () -> {
          io.setAngle(Rotations.of(IntakeTiltConstants.IN_POSITION));
        });
  }
}
