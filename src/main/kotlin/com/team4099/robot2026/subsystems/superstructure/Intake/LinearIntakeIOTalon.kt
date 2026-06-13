import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2026.config.constants.Constants
import edu.wpi.first.units.measure.Angle as WPIAngle
import edu.wpi.first.units.measure.AngularAcceleration as WPIAngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity as WPIAngularVelocity
import edu.wpi.first.units.measure.Current as WPICurrent
import edu.wpi.first.units.measure.Temperature as WPITemperature
import edu.wpi.first.units.measure.Voltage as WPIVoltage
import org.team4099.lib.units.ctreLinearMechanismSensor
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.inInchesPerSecondPerSecond


object LinearIntakeIOTalon: LinearIntakeIO {
  private val LintakeTalon: TalonFX = TalonFX(Constants.Lintake.LINTAKE_MOTOR_ID) //Create Motor
  private val motionMagicControl: MotionMagicVoltage = MotionMagicVoltage(-1337.degrees.inDegrees)
  private val configs: TalonFXConfiguration = TalonFXConfiguration()
  private val LintakeSensor = ctreLinearMechanismSensor(
    LintakeTalon,
    IntakeConstants.LinearIntakeConstants.GEAR_RATIO,
    IntakeConstants.LinearIntakeConstants.DIAMETER,
    IntakeConstants.LinearIntakeConstants.VOLTAGE_COMPENSATION
  )

  var statorCurrentSignal: StatusSignal<WPICurrent>
  var supplyCurrentSignal: StatusSignal<WPICurrent>
  var tempSignal: StatusSignal<WPITemperature>
  var dutyCycleSignal: StatusSignal<Double>
  var motorTorqueSignal: StatusSignal<WPICurrent>
  var motorVoltageSignal: StatusSignal<WPIVoltage>
  var motorAccelSignal: StatusSignal<WPIAngularAcceleration>
  var rotorPositionSignal: StatusSignal<WPIAngle>
  var rotorVelocitySignal: StatusSignal<WPIAngularVelocity>
  init {
    LintakeTalon.clearStickyFaults()


    configs.CurrentLimits.SupplyCurrentLimit = 40.0
    configs.CurrentLimits.StatorCurrentLimit = 40.0
    configs.CurrentLimits.SupplyCurrentLimitEnable = true
    configs.CurrentLimits.StatorCurrentLimitEnable = true
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake


    configs.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.LinearIntakeConstants.MAX_VELOCITY.inInchesPerSecond
    configs.MotionMagic.MotionMagicAcceleration = IntakeConstants.LinearIntakeConstants.MAX_ACCELERATION.inInchesPerSecondPerSecond

    LintakeTalon.configurator.apply(configs)
    rotorPositionSignal = LintakeTalon.position
    rotorVelocitySignal = LintakeTalon.velocity
    statorCurrentSignal = LintakeTalon.statorCurrent
    supplyCurrentSignal = LintakeTalon.supplyCurrent
    tempSignal = LintakeTalon.deviceTemp
    dutyCycleSignal = LintakeTalon.dutyCycle
    motorTorqueSignal = LintakeTalon.torqueCurrent
    motorVoltageSignal = LintakeTalon.motorVoltage
    motorAccelSignal = LintakeTalon.acceleration

    zeroEncoder()
  }

}