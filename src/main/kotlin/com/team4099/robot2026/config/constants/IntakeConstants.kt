import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object IntakeConstants {
  object LintakePID {
    val SIM_KP = 0.0.volts / 1.inches
    val SIM_KI = 0.0.volts / (1.inches * 1.seconds)
    val SIM_KD = 0.0.volts / (1.inches.perSecond)

    val REAL_KP = 0.0.volts / 1.inches
    val REAL_KI = 0.0.volts / (1.inches * 1.seconds)
    val REAL_KD = 0.0.volts / (1.inches.perSecond)

    val SIM_KS = 0.0.volts

    val REAL_KS = 0.0.volts
  }

  object LinearIntakeConstants {
    val CURRENT_LIMIT = 40.0.amps
    val START_POSITION = 0.0.inches
    val MANUAL_ZERO_POSITION = 0.inches
    val IDLE_VOLTAGE = 0.0.volts
    val GEAR_RATIO: Double = 1.0 / 1.0
    val VOLTAGE_COMPENSATION = 12.0.volts
    val DIAMETER = 1.0.inches
    val MAX_ACCELERATION = 0.0.inches.perSecond.perSecond
    val MAX_VELOCITY = 0.0.inches.perSecond
    val LINTAKE_MASS = 0.0.grams
    val FORWARD_EXTENSION_LIM = 0.0.inches
    val BACKMOST_EXTENSION_LIM = 0.0.inches
  }

  object RollerIntakeConstants {
    val IDLE_VOLTAGE = 0.0.volts
    val STATOR_CURRENT_LIMIT = 0.0.amps
    val SUPPLY_CURRENT_LIMIT = 0.0.amps
    val GEAR_RATIO = 1.0 / 1.0
    val VOLTAGE_COMPENSATION = 12.0.volts
    val MOMENT_OF_INERTIA = 1.0.grams.meterSquared
  }
}
