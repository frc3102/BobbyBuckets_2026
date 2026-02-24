package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.team3061.sim.ElevatorSystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOTalonFX implements ElevatorIO {

  private final TalonFX elevator = new TalonFX(ElevatorConstants.CAN_ID);
  private final StatusSignal<Voltage> appliedVolts = elevator.getMotorVoltage();
  private final StatusSignal<Current> statorCurrent = elevator.getStatorCurrent();
  private final StatusSignal<Current> supplyCurrent = elevator.getSupplyCurrent();
  private final StatusSignal<Angle> position = elevator.getPosition();
  private final StatusSignal<AngularVelocity> velocity = elevator.getVelocity();
  private final StatusSignal<Temperature> temp = elevator.getDeviceTemp();
  private final StatusSignal<Double> closedLoopError = elevator.getClosedLoopError();
  private final StatusSignal<Double> closedLoopReference = elevator.getClosedLoopReference();

  private final LoggedTunableNumber motorKP =
      new LoggedTunableNumber("Elevator/kP", ElevatorConstants.Motor.KP);
  private final LoggedTunableNumber motorKI =
      new LoggedTunableNumber("Elevator/kI", ElevatorConstants.Motor.KI);
  private final LoggedTunableNumber motorKD =
      new LoggedTunableNumber("Elevator/kD", ElevatorConstants.Motor.KD);

  private final LoggedTunableNumber motorKS =
      new LoggedTunableNumber("Elevator/kS", ElevatorConstants.Motor.KS);
  private final LoggedTunableNumber motorKG =
      new LoggedTunableNumber("Elevator/kG", ElevatorConstants.Motor.KG);
  private final LoggedTunableNumber motorKV =
      new LoggedTunableNumber("Elevator/kV", ElevatorConstants.Motor.KV);
  private final LoggedTunableNumber motorKA =
      new LoggedTunableNumber("Elevator/kA", ElevatorConstants.Motor.KA);
  private final LoggedTunableNumber motorKVExpo =
      new LoggedTunableNumber("Elevator/kV_Expo", ElevatorConstants.Motor.KV_EXPO);
  private final LoggedTunableNumber motorKAExpo =
      new LoggedTunableNumber("Elevator/kA_Expo", ElevatorConstants.Motor.KA_EXPO);

  private final ElevatorSystemSim elevatorSim;

  private final DynamicMotionMagicExpoVoltage positionRequest;
  private final VoltageOut voltageRequest;

  public ElevatorIOTalonFX() {

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        appliedVolts,
        statorCurrent,
        supplyCurrent,
        position,
        velocity,
        temp,
        closedLoopError,
        closedLoopReference);
    ParentDevice.optimizeBusUtilizationForAll(elevator);

    var config = new TalonFXConfiguration();
    config.Slot0.kP = motorKP.get();
    config.Slot0.kI = motorKI.get();
    config.Slot0.kD = motorKD.get();
    config.Slot0.kA = motorKA.get();
    config.Slot0.kS = motorKS.get();
    config.Slot0.kG = motorKG.get();
    config.Slot0.kV = motorKV.get();
    config.MotionMagic.MotionMagicExpo_kA = motorKAExpo.get();
    config.MotionMagic.MotionMagicExpo_kV = motorKVExpo.get();
    config.MotionMagic.MotionMagicCruiseVelocity = 0;
    PhoenixUtil.tryUntilOk(5, () -> elevator.getConfigurator().apply(config, 0.25));
    positionRequest = new DynamicMotionMagicExpoVoltage(0, motorKVExpo.get(), motorKAExpo.get());
    voltageRequest = new VoltageOut(0);

    zeroElevator();

    elevatorSim =
        new ElevatorSystemSim(
            elevator,
            false,
            ElevatorConstants.GEAR_RATIO,
            ElevatorConstants.Mechanism.ELEVATOR_MASS.in(Kilograms),
            ElevatorConstants.Mechanism.PULLEY_CIRCUMFERENCE.in(Meters),
            ElevatorConstants.Mechanism.MIN_HEIGHT.in(Meters),
            ElevatorConstants.Mechanism.MAX_HEIGHT.in(Meters),
            ElevatorConstants.Mechanism.MIN_HEIGHT.in(Meters),
            "Elevator");
  }

  @Override
  public void setPosition(Distance height) {
    elevator.setControl(
        positionRequest.withPosition(
            height.in(Inches)
                * ElevatorConstants.GEAR_RATIO
                / ElevatorConstants.Mechanism.PULLEY_CIRCUMFERENCE.in(Inches)));
  }

  @Override
  public void setVoltage(Voltage volts) {
    elevator.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            position, velocity, appliedVolts, statorCurrent, supplyCurrent, temp);
    inputs.connected = status.isOK();
    inputs.angularPosition = position.getValue();
    inputs.appliedVolts = appliedVolts.getValue();
    inputs.linearPosition =
        Inches.of(
            position.getValueAsDouble()
                * ElevatorConstants.Mechanism.PULLEY_CIRCUMFERENCE.in(Inches)
                / ElevatorConstants.GEAR_RATIO);
    inputs.statorCurrentAmps = statorCurrent.getValue();
    inputs.supplyCurrentAmps = supplyCurrent.getValue();
    inputs.temp = temp.getValue();
    inputs.velocity = velocity.getValue();

    if (Constants.TUNING_MODE) {
      inputs.closedLoopReference = closedLoopReference.getValueAsDouble();
      inputs.closedLoopError = closedLoopError.getValueAsDouble();
    }

    if (Constants.getMode() == Constants.Mode.SIM) {
      this.elevatorSim.updateSim();
    }
  }

  @Override
  public void zeroElevator() {
    elevator.setPosition(0);
  }
}
