package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3061.sim.VelocitySystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class SuperstructureIOTalonFX implements SuperstructureIO {
  private TalonFX conveyor = new TalonFX(SuperstructureConstants.Conveyor.CAN_ID);
  private TalonFX kicker = new TalonFX(SuperstructureConstants.Kicker.CAN_ID);
  private TalonFX shooterLeader = new TalonFX(SuperstructureConstants.Shooter.CAN_ID_LEADER);
  private TalonFX shooterFollower = new TalonFX(SuperstructureConstants.Shooter.CAN_ID_FOLLOWER);

  // Conveyor Tunables
  private LoggedTunableNumber conveyorKP =
      new LoggedTunableNumber(
          "Superstructure/Conveyor/KP", SuperstructureConstants.Conveyor.Motor.KP);
  private LoggedTunableNumber conveyorKI =
      new LoggedTunableNumber(
          "Superstructure/Conveyor/KI", SuperstructureConstants.Conveyor.Motor.KI);
  private LoggedTunableNumber conveyorKD =
      new LoggedTunableNumber(
          "Superstructure/Conveyor/KD", SuperstructureConstants.Conveyor.Motor.KD);
  private LoggedTunableNumber conveyorKS =
      new LoggedTunableNumber(
          "Superstructure/Conveyor/KS", SuperstructureConstants.Conveyor.Motor.KS);
  private LoggedTunableNumber conveyorKV =
      new LoggedTunableNumber(
          "Superstructure/Conveyor/KV", SuperstructureConstants.Conveyor.Motor.KV);
  private LoggedTunableNumber conveyorKA =
      new LoggedTunableNumber(
          "Superstructure/Conveyor/KA", SuperstructureConstants.Conveyor.Motor.KA);
  private LoggedTunableNumber conveyorMMAccel =
      new LoggedTunableNumber(
          "Superstructure/Conveyor/MM_A", SuperstructureConstants.Conveyor.Motor.MM_A);
  private LoggedTunableNumber conveyorMMJerk =
      new LoggedTunableNumber(
          "Superstructure/Conveyor/MM_JERK", SuperstructureConstants.Conveyor.Motor.MM_JERK);
  // Kicker Tunables
  private LoggedTunableNumber kickerKP =
      new LoggedTunableNumber("Superstructure/Kicker/KP", SuperstructureConstants.Kicker.Motor.KP);
  private LoggedTunableNumber kickerKI =
      new LoggedTunableNumber("Superstructure/Kicker/KI", SuperstructureConstants.Kicker.Motor.KI);
  private LoggedTunableNumber kickerKD =
      new LoggedTunableNumber("Superstructure/Kicker/KD", SuperstructureConstants.Kicker.Motor.KD);
  private LoggedTunableNumber kickerKS =
      new LoggedTunableNumber("Superstructure/Kicker/KS", SuperstructureConstants.Kicker.Motor.KS);
  private LoggedTunableNumber kickerKV =
      new LoggedTunableNumber("Superstructure/Kicker/KV", SuperstructureConstants.Kicker.Motor.KV);
  private LoggedTunableNumber kickerKA =
      new LoggedTunableNumber("Superstructure/Kicker/KA", SuperstructureConstants.Kicker.Motor.KA);
  private LoggedTunableNumber kickerMMAccel =
      new LoggedTunableNumber(
          "Superstructure/Kicker/MM_A", SuperstructureConstants.Kicker.Motor.MM_A);
  private LoggedTunableNumber kickerMMJerk =
      new LoggedTunableNumber(
          "Superstructure/Kicker/MM_JERK", SuperstructureConstants.Kicker.Motor.MM_JERK);
  // Shooter Tunables
  private LoggedTunableNumber shooterKP =
      new LoggedTunableNumber(
          "Superstructure/Shooter/KP", SuperstructureConstants.Shooter.Motor.KP);
  private LoggedTunableNumber shooterKI =
      new LoggedTunableNumber(
          "Superstructure/Shooter/KI", SuperstructureConstants.Shooter.Motor.KI);
  private LoggedTunableNumber shooterKD =
      new LoggedTunableNumber(
          "Superstructure/Shooter/KD", SuperstructureConstants.Shooter.Motor.KD);
  private LoggedTunableNumber shooterKS =
      new LoggedTunableNumber(
          "Superstructure/Shooter/KS", SuperstructureConstants.Shooter.Motor.KS);
  private LoggedTunableNumber shooterKV =
      new LoggedTunableNumber(
          "Superstructure/Shooter/KV", SuperstructureConstants.Shooter.Motor.KV);
  private LoggedTunableNumber shooterKA =
      new LoggedTunableNumber(
          "Superstructure/Shooter/KA", SuperstructureConstants.Shooter.Motor.KA);
  private LoggedTunableNumber shooterMMAccel =
      new LoggedTunableNumber(
          "Superstructure/Shooter/MM_A", SuperstructureConstants.Shooter.Motor.MM_A);
  private LoggedTunableNumber shooterMMJerk =
      new LoggedTunableNumber(
          "Superstructure/Shooter/MM_JERK", SuperstructureConstants.Shooter.Motor.MM_JERK);

  // Alerts
  private Alert conveyorConfigAlert =
      new Alert("Failed to apply conveyor config", AlertType.kError);
  private Alert kickerConfigAlert = new Alert("Failed to apply conveyor config", AlertType.kError);
  private Alert shooterLeaderConfigAlert =
      new Alert("Failed to apply conveyor config", AlertType.kError);
  private Alert shooterFollowerConfigAlert =
      new Alert("Failed to apply conveyor config", AlertType.kError);

  // Conveyor Signals
  private StatusSignal<Voltage> conveyorVolts = conveyor.getMotorVoltage();
  private StatusSignal<Current> conveyorSupplyAmps = conveyor.getSupplyCurrent();
  private StatusSignal<AngularVelocity> conveyorVelocity = conveyor.getVelocity();
  private StatusSignal<Temperature> conveyorTemp = conveyor.getDeviceTemp();

  // Kicker Signals
  private StatusSignal<Voltage> kickerVolts = kicker.getMotorVoltage();
  private StatusSignal<Current> kickerSupplyAmps = kicker.getSupplyCurrent();
  private StatusSignal<AngularVelocity> kickerVelocity = kicker.getVelocity();
  private StatusSignal<Temperature> kickerTemp = kicker.getDeviceTemp();

  // Shooter Leader Signals
  private StatusSignal<Voltage> shooterLeaderVolts = shooterLeader.getMotorVoltage();
  private StatusSignal<Current> shooterLeaderSupplyAmps = shooterLeader.getSupplyCurrent();
  private StatusSignal<AngularVelocity> shooterLeaderVelocity = shooterLeader.getVelocity();
  private StatusSignal<Temperature> shooterLeaderTemp = shooterLeader.getDeviceTemp();

  private AngularVelocity shooterTargetVelocity = RotationsPerSecond.of(0);
  // Shooter Follower Signals
  private StatusSignal<Voltage> shooterFollowerVolts = shooterFollower.getMotorVoltage();
  private StatusSignal<Current> shooterFollowerSupplyAmps = shooterFollower.getSupplyCurrent();
  private StatusSignal<Temperature> shooterFollowerTemp = shooterFollower.getDeviceTemp();

  // Control signals
  private MotionMagicVelocityVoltage conveyorVoltage = new MotionMagicVelocityVoltage(0);
  private MotionMagicVelocityVoltage kickerVoltage = new MotionMagicVelocityVoltage(0);
  private MotionMagicVelocityVoltage shooterVoltage = new MotionMagicVelocityVoltage(0);

  private final VelocitySystemSim conveyorSim;
  private final VelocitySystemSim kickerSim;
  private final VelocitySystemSim shooterSim;

  public SuperstructureIOTalonFX() {

    configConveyor(conveyor);
    configKicker(kicker);
    configShooterMotor(shooterLeader, shooterLeaderConfigAlert);
    configShooterMotor(shooterFollower, shooterFollowerConfigAlert);

    shooterFollower.setControl(
        new Follower(SuperstructureConstants.Shooter.CAN_ID_LEADER, MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        conveyorSupplyAmps,
        conveyorVolts,
        conveyorTemp,
        conveyorVelocity,
        kickerSupplyAmps,
        kickerVolts,
        kickerTemp,
        kickerVelocity,
        shooterLeaderSupplyAmps,
        shooterLeaderVolts,
        shooterLeaderTemp,
        shooterLeaderVelocity,
        shooterFollowerSupplyAmps,
        shooterFollowerVolts,
        shooterFollowerTemp);
    ParentDevice.optimizeBusUtilizationForAll(conveyor, kicker, shooterLeader, shooterFollower);
    conveyorSim =
        new VelocitySystemSim(
            conveyor,
            false,
            SuperstructureConstants.Conveyor.Motor.KV,
            SuperstructureConstants.Conveyor.Motor.KA,
            1);
    kickerSim =
        new VelocitySystemSim(
            kicker,
            false,
            SuperstructureConstants.Kicker.Motor.KV,
            SuperstructureConstants.Kicker.Motor.KA,
            1);
    shooterSim =
        new VelocitySystemSim(
            shooterLeader,
            false,
            SuperstructureConstants.Shooter.Motor.KV,
            SuperstructureConstants.Shooter.Motor.KA,
            1);
  }

  private void configConveyor(TalonFX motor) {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = SuperstructureConstants.Conveyor.Motor.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = conveyorKP.get();
    config.Slot0.kI = conveyorKI.get();
    config.Slot0.kD = conveyorKD.get();
    config.Slot0.kS = conveyorKS.get();
    config.Slot0.kG = 0;
    config.Slot0.kV = conveyorKV.get();
    config.Slot0.kA = conveyorKA.get();

    config.MotionMagic.MotionMagicAcceleration = conveyorMMAccel.get();
    config.MotionMagic.MotionMagicJerk = conveyorMMJerk.get();
    Phoenix6Util.applyAndCheckConfiguration(motor, config, conveyorConfigAlert);
  }

  private void configKicker(TalonFX motor) {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = SuperstructureConstants.Kicker.Motor.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = kickerKP.get();
    config.Slot0.kI = kickerKI.get();
    config.Slot0.kD = kickerKD.get();
    config.Slot0.kS = kickerKS.get();
    config.Slot0.kG = 0;
    config.Slot0.kV = kickerKV.get();
    config.Slot0.kA = kickerKA.get();

    config.MotionMagic.MotionMagicAcceleration = kickerMMAccel.get();
    config.MotionMagic.MotionMagicJerk = kickerMMJerk.get();
    Phoenix6Util.applyAndCheckConfiguration(motor, config, kickerConfigAlert);
  }

  private void configShooterMotor(TalonFX motor, Alert alert) {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = SuperstructureConstants.Conveyor.Motor.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = shooterKP.get();
    config.Slot0.kI = shooterKI.get();
    config.Slot0.kD = shooterKD.get();
    config.Slot0.kS = shooterKS.get();
    config.Slot0.kG = 0;
    config.Slot0.kV = shooterKV.get();
    config.Slot0.kA = shooterKA.get();

    config.MotionMagic.MotionMagicAcceleration = shooterMMAccel.get();
    config.MotionMagic.MotionMagicJerk = shooterMMJerk.get();
    Phoenix6Util.applyAndCheckConfiguration(motor, config, alert);
  }

  @Override
  public void setShooterRPS(AngularVelocity velocity) {
    shooterTargetVelocity = velocity.copy();
    shooterLeader.setControl(shooterVoltage.withVelocity(velocity));
  }

  @Override
  public void setConveyorRPS(AngularVelocity velocity) {
    conveyor.setControl(conveyorVoltage.withVelocity(velocity));
  }

  @Override
  public void setKickerRPS(AngularVelocity velocity) {
    kicker.setControl(kickerVoltage.withVelocity(velocity));
  }

  @Override
  public void updateInputs(SuperstructureIOInputs inputs) {
    boolean conveyorConnected =
        BaseStatusSignal.refreshAll(
                conveyorSupplyAmps, conveyorVolts, conveyorTemp, conveyorVelocity)
            .isOK();
    boolean kickerConnected =
        BaseStatusSignal.refreshAll(kickerSupplyAmps, kickerVolts, kickerTemp, kickerVelocity)
            .isOK();
    boolean shooterLeaderConnected =
        BaseStatusSignal.refreshAll(
                shooterLeaderSupplyAmps,
                shooterLeaderVolts,
                shooterLeaderTemp,
                shooterLeaderVelocity)
            .isOK();
    boolean shooterFollowerConnected =
        BaseStatusSignal.refreshAll(
                shooterFollowerSupplyAmps, shooterFollowerVolts, shooterFollowerTemp)
            .isOK();

    inputs.conveyorConnected = conveyorConnected;
    inputs.kickerConnected = kickerConnected;
    inputs.shooterLeaderConnected = shooterLeaderConnected;
    inputs.shooterFollowerConnected = shooterFollowerConnected;

    inputs.conveyorVolts = conveyorVolts.getValue();
    inputs.kickerVolts = kickerVolts.getValue();
    inputs.shooterLeaderVolts = shooterLeaderVolts.getValue();
    inputs.shooterFollowerVolts = shooterFollowerVolts.getValue();

    inputs.conveyorAmps = conveyorSupplyAmps.getValue();
    inputs.kickerAmps = kickerSupplyAmps.getValue();
    inputs.shooterLeaderAmps = shooterLeaderSupplyAmps.getValue();
    inputs.shooterFollowerAmps = shooterFollowerSupplyAmps.getValue();

    inputs.conveyorVelocity = conveyorVelocity.getValue();
    inputs.kickerVelocity = kickerVelocity.getValue();
    inputs.shooterVelocity = shooterLeaderVelocity.getValue();
    inputs.shooterTargetVelocity = shooterTargetVelocity.copy();

    if (Constants.getMode() == Mode.SIM) {
      conveyorSim.updateSim();
      kickerSim.updateSim();
      shooterSim.updateSim();
    }
  }
}
