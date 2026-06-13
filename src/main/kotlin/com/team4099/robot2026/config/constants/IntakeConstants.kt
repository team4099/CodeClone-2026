import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.volts

object IntakeConstants{
  object LinearIntakeConstants{
    val START_POSITION = 0.0.inches
    val IDLE_VOLTAGE = 0.0.volts
    val GEAR_RATIO: Double = 1.0 / 1.0
    val VOLTAGE_COMPENSATION = 12.0.volts
    val DIAMETER = 1.0.inches
  }
  object RollerIntakeConstants{
    val IDLE_VOLTAGE = 0.0.volts
  }
}
