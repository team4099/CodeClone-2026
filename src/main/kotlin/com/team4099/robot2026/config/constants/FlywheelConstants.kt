package com.team4099.robot2026.config.constants

import org.team4099.lib.units.AngularAcceleration
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo
import org.team4099.lib.units.perSecond

object FlywheelConstants {

  val GEAR_RATIO = 4.0 / 1.0
  val SUPPLY_CURRENT_LIMIT = 40.0.amps
  val STATOR_CURRENT_LIMIT = 80.0.amps
  val IDLE_VOLTAGE = 0.0.volts
  val VOLTAGE_COMPENSATION = 12.0.volts
  val MOMENT_OF_INERTIA = 0.0067.kilo.grams.meterSquared
  val MAX_ACCELERATION: AngularAcceleration = 1000.rotations.perSecond.perSecond
}
