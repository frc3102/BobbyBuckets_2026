package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.team3061.sim.ArmSystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class TurretIOTalonFX implements TurretIO {

  private final TalonFX turret = new TalonFX(TurretConstants.CAN_ID);
  private final StatusSignal<Angle> positionRot = turret.getPosition();
  private final StatusSignal<AngularVelocity> velocityRotPerSec = turret.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = turret.getMotorVoltage();
  private final StatusSignal<Current> currentAmps = turret.getSupplyCurrent();
  private final StatusSignal<Temperature> currentTemp = turret.getDeviceTemp();

  private final PositionVoltage positionRequest;
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final LoggedTunableNumber motorKP =
      new LoggedTunableNumber("Turret/kP", TurretConstants.Motor.KP);
  private final LoggedTunableNumber motorKI =
      new LoggedTunableNumber("Turret/kI", TurretConstants.Motor.KI);
  private final LoggedTunableNumber motorKD =
      new LoggedTunableNumber("Turret/kD", TurretConstants.Motor.KD);
  private final LoggedTunableNumber motorKS =
      new LoggedTunableNumber("Turret/kS", TurretConstants.Motor.KS);
  private final LoggedTunableNumber motorKV =
      new LoggedTunableNumber("Turret/kV", TurretConstants.Motor.KV);
  private final LoggedTunableNumber motorKA =
      new LoggedTunableNumber("Turret/kA", TurretConstants.Motor.KA);
  private final ArmSystemSim turretSim;

  public TurretIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = TurretConstants.Motor.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Slot0.kP = motorKP.get();
    config.Slot0.kI = motorKI.get();
    config.Slot0.kD = motorKD.get();
    config.Slot0.kS = motorKS.get();
    config.Slot0.kV = motorKV.get();
    config.Slot0.kA = motorKA.get();

    PhoenixUtil.tryUntilOk(5, () -> turret.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps, currentTemp);
    ParentDevice.optimizeBusUtilizationForAll(turret);

    positionRequest = new PositionVoltage(0).withFeedForward(2).withVelocity(1);
    zeroPosition();
    Logger.recordOutput("Turret/TargetAngle", Degrees.of(0).magnitude());

    turretSim =
        new ArmSystemSim(
            turret,
            false,
            TurretConstants.GEAR_RATIO,
            Inches.of(12).in(Meters),
            Pounds.of(5).in(Kilograms),
            TurretConstants.MIN_ANGLE.in(Radians),
            TurretConstants.MAX_ANGLE.in(Radians),
            0,
            "Turret");
  }

  @Override
  public void setAngle(Angle angle) {
    if (angle.gt(TurretConstants.MAX_ANGLE)) {
      angle = TurretConstants.MAX_ANGLE;
    } else if (angle.lt(TurretConstants.MIN_ANGLE)) {
      angle = TurretConstants.MIN_ANGLE;
    }
    Logger.recordOutput("Turret/TargetAngle", angle.in(Degrees));
    turret.setControl(positionRequest.withPosition(angle.times(TurretConstants.GEAR_RATIO)));
  }

  @Override
  public void setVoltage(Voltage volts) {
    turret.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stepAngle(Angle step) {
    var currentPos = turret.getPosition().getValue();
    var targetPos = currentPos.plus(step.times(TurretConstants.GEAR_RATIO));
    this.setAngle(targetPos);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                positionRot, velocityRotPerSec, appliedVolts, currentAmps, currentTemp)
            .isOK();
    inputs.position = positionRot.getValue();
    inputs.appliedVolts = appliedVolts.getValue();
    inputs.currentAmps = currentAmps.getValue();
    inputs.temp = currentTemp.getValue();

    LoggedTunableNumber.ifChanged(
        hashCode(),
        motionMagic -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.turret.getConfigurator().refresh(config);
          config.Slot0.kP = motionMagic[0];
          config.Slot0.kI = motionMagic[1];
          config.Slot0.kD = motionMagic[2];
          config.Slot0.kS = motionMagic[3];
          config.Slot0.kV = motionMagic[4];
          config.Slot0.kA = motionMagic[5];
          this.turret.getConfigurator().apply(config);
        },
        motorKP,
        motorKI,
        motorKD,
        motorKS,
        motorKV,
        motorKA);

    if (Constants.getMode() == Constants.Mode.SIM) {
      this.turretSim.updateSim();
    }
  }

  @Override
  public void zeroPosition() {
    turret.setPosition(0);
  }
}
