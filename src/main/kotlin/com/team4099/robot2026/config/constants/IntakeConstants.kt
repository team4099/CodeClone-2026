import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond


object IntakeConstants{
  object LinearIntakeConstants{
    val START_POSITION = 0.0.inches
    val MANUAL_ZERO_POSITION = 0.inches
    val IDLE_VOLTAGE = 0.0.volts
    val GEAR_RATIO: Double = 1.0 / 1.0
    val VOLTAGE_COMPENSATION = 12.0.volts
    val DIAMETER = 1.0.inches
    val MAX_ACCELERATION = 0.0.inches.perSecond.perSecond
    val MAX_VELOCITY = 0.0.inches.perSecond
  }
  object RollerIntakeConstants{
    val IDLE_VOLTAGE = 0.0.volts
    val STATOR_CURRENT_LIMIT = 0.0.amps
    val SUPPLY_CURRENT_LIMIT = 0.0.amps
    val GEAR_RATIO = 1.0 / 1.0
    val VOLTAGE_COMPENSATION = 12.0.volts
    val MOMENT_OF_INERTIA = 1.0.grams.meterSquared
  }
}
