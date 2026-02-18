package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTracer;
import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {
  private final LauncherIO io;
  private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

  private final SysIdRoutine sysIdRoutine;

  public Launcher(LauncherIO io) {
    this.io = io;

    this.sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(5).per(Second), // will be 5 amps/sec
                Volts.of(10), // start at 10A
                Seconds.of(5), // use default timeout (10 s)
                state -> SignalLogger.writeString("SysId_State", state.toString())),
            new SysIdRoutine.Mechanism(
                output -> io.setCurrent(Amps.of(output.in(Volts))), null, this));
    SysIdRoutineChooser.getInstance().addOption("Launcher Voltage", sysIdRoutine);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Launcher", inputs);
    LoggedTracer.record("Launcher");
  }

  public Command stopLauncher() {
    return runOnce(
        () -> {
          io.setTargetVelocity(RotationsPerSecond.of(0));
        });
  }

  public Command startAtSpeed(AngularVelocity speed) {
    return runOnce(
        () -> {
          io.setTargetVelocity(speed);
        });
  }

  public Command startAtDistance(Distance distance) {
    return runOnce(
        () -> {
          io.setTargetVelocity(speedForDistance(distance));
        });
  }

  private AngularVelocity speedForDistance(Distance distance) {
    // TODO: make a LUT or lerp for distance-to-target
    return RotationsPerSecond.of(3000.0 / 60.0);
  }
}
