package frc.robot.subsystems.intake.tilt;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeTiltIO {

  @AutoLog
  public static class IntakeTiltIOInputs {
    public boolean connected = false;
    public double velocityRadPerSec = 0.0;
    public double positionRad = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(IntakeTiltIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
