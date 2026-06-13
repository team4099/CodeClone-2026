import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.team4099.robot2026.config.constants.Constants
import org.team4099.lib.units.ctreLinearMechanismSensor
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees

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
}