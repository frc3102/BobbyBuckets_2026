package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTracer;
import frc.robot.subsystems.elevator.ElevatorConstants.Positions;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private Positions targetPosition = Positions.BOTTOM;

  private final Debouncer atSetpointDebouncer = new Debouncer(0.1);

  private final SysIdRoutine sysIdRoutine;

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.zeroElevator();
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(2.0).per(Second), // override default ramp rate (1 V/s)
                Volts.of(2.0), // override default step voltage (7 V)
                null, // Use default timeout (10 s)
                state -> SignalLogger.writeString("SysId_State", state.toString())),
            new SysIdRoutine.Mechanism(output -> io.setVoltage(output), null, this));

    SysIdRoutineChooser.getInstance().addOption("Elevator Voltage", sysIdRoutine);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/targetPosition", targetPosition);
    LoggedTracer.record("Elevator");
  }

  public Distance getPosition() {
    return inputs.linearPosition;
  }

  public boolean isAtPosition(Positions position) {
    return atSetpointDebouncer.calculate(
        getPosition()
            .isNear(position.getHeight(), ElevatorConstants.Mechanism.LINEAR_POSITION_TOLERANCE));
  }

  public Command goToPosition(Positions position) {
    targetPosition = position;
    return runOnce(
        () -> {
          io.setPosition(targetPosition.getHeight());
        });
  }

  public Command stop() {
    return runOnce(
        () -> {
          io.setVoltage(Volts.of(0));
        });
  }

  public Command zeroElevator() {
    return runOnce(
        () -> {
          io.zeroElevator();
        });
  }
}
