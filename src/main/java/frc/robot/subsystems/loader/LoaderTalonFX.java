package frc.robot.subsystems.loader;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.team6328.util.LoggedTunableNumber;

public class LoaderTalonFX implements LoaderIO {
  private final TalonFX intake = new TalonFX(LoaderConstants.CAN_ID);
  private final StatusSignal<Angle> intakePositionRot = intake.getPosition();
  private final StatusSignal<AngularVelocity> intakeVelocityRotPerSec = intake.getVelocity();
  private final StatusSignal<Voltage> intakeAppliedVolts = intake.getMotorVoltage();
  private final StatusSignal<Current> intakeCurrentAmps = intake.getSupplyCurrent();
  private final StatusSignal<Temperature> intakeCurrentTemp = intake.getDeviceTemp();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  private final LoggedTunableNumber motorKP =
      new LoggedTunableNumber("Loader/kP", LoaderConstants.Motor.KP);
  private final LoggedTunableNumber motorKI =
      new LoggedTunableNumber("Loader/kI", LoaderConstants.Motor.KI);
  private final LoggedTunableNumber motorKD =
      new LoggedTunableNumber("Loader/kD", LoaderConstants.Motor.KD);

  private final LoggedTunableNumber motorKS =
      new LoggedTunableNumber("Loader/kS", LoaderConstants.Motor.KS);
  private final LoggedTunableNumber motorKG =
      new LoggedTunableNumber("Loader/kG", LoaderConstants.Motor.KG);
  private final LoggedTunableNumber motorKV =
      new LoggedTunableNumber("Loader/kV", LoaderConstants.Motor.KV);
  private final LoggedTunableNumber motorKA =
      new LoggedTunableNumber("Loader/kA", LoaderConstants.Motor.KA);

  public LoaderTalonFX() {
    var intakeConfig = new TalonFXConfiguration();
    intakeConfig.CurrentLimits.SupplyCurrentLimit = LoaderConstants.Motor.CURRENT_LIMIT;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeConfig.Slot0.kP = motorKP.get();
    intakeConfig.Slot0.kI = motorKI.get();
    intakeConfig.Slot0.kD = motorKD.get();

    intakeConfig.Slot0.kS = motorKS.get();
    intakeConfig.Slot0.kG = motorKG.get();
    intakeConfig.Slot0.kV = motorKV.get();
    intakeConfig.Slot0.kA = motorKA.get();
    tryUntilOk(5, () -> intake.getConfigurator().apply(intakeConfig, 0.25));
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        intakePositionRot,
        intakeVelocityRotPerSec,
        intakeAppliedVolts,
        intakeCurrentAmps,
        intakeCurrentTemp);
    ParentDevice.optimizeBusUtilizationForAll(intake);
  }

  @Override
  public void setVoltage(Voltage volts) {
    intake.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void updateInputs(LoaderIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            intakePositionRot, intakeVelocityRotPerSec, intakeAppliedVolts, intakeCurrentAmps);
    inputs.connected = status.isOK();
    inputs.velocity = intakeVelocityRotPerSec.getValue().div(LoaderConstants.GEAR_RATIO);
    inputs.appliedVolts = intakeAppliedVolts.getValue();
    inputs.currentAmps = intakeCurrentAmps.getValue();
    inputs.temp = intakeCurrentTemp.getValue();
    LoggedTunableNumber.ifChanged(
        hashCode(),
        motionMagic -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.intake.getConfigurator().refresh(config);
          config.Slot0.kP = motionMagic[0];
          config.Slot0.kI = motionMagic[1];
          config.Slot0.kD = motionMagic[2];
          config.Slot0.kS = motionMagic[3];
          config.Slot0.kG = motionMagic[4];
          config.Slot0.kA = motionMagic[5];
          config.Slot0.kV = motionMagic[6];
          this.intake.getConfigurator().apply(config);
        },
        motorKP,
        motorKI,
        motorKD,
        motorKS,
        motorKG,
        motorKA,
        motorKV);
  }
}
