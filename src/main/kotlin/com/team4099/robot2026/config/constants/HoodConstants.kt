package com.team4099.robot2026.config.constants


import org.team4099.lib.units.AngularAcceleration
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo
import org.team4099.lib.units.perSecond

object HoodConstants {
  val GEAR_RATIO = 4.0/1.0
  val SUPPLY_CURRENT_LIMIT = 40.0.amps
  val STATOR_CURRENT_LIMIT = 80.0.amps
  val IDLE_VOLTAGE = 0.0.volts
  val VOLTAGE_COMPENSATION = 12.0.volts
  val MOMENT_OF_INERTIA = 0.0067.kilo.grams.meterSquared
  val MAX_ACCELERATION: AngularAcceleration = 1000.rotations.perSecond.perSecond




  object PID {
    val REAL_KP = 1.0.volts /1.radians.perSecond
    val REAL_KI = 0.0.volts / (1.radians.perSecond * 1.seconds)
    val REAL_KD = 0.0.volts/ (1.radians.perSecond / 1. seconds)

    val SIM_KP  = 1.0.volts /1.radians.perSecond
    val SIM_KI = 0.0.volts / (1.radians.perSecond * 1.seconds)
    val SIM_KD = 0.0.volts/ (1.radians.perSecond / 1. seconds)


    val SIM_KS = 0.0.volts
    val SIM_KV = (1.0 / 3000.0).volts / 1.radians.perSecond
    val SIM_KA = 0.0.volts / 1.radians.perSecond.perSecond

    val REAL_KS = 0.0.volts
    val REAL_KV = (1.0 / 3000.0).volts / 1.radians.perSecond
    val REAL_KA = 0.0.volts / 1.radians.perSecond.perSecond
  }

  /*
   kP: ProportionalGain<Fraction<Radian, Second>, Volt>,
      kI: IntegralGain<Fraction<Radian, Second>, Volt>,
      kD: DerivativeGain<Fraction<Radian, Second>, Volt>
    ) {}

    fun configureFFVoltage(
      kS: StaticFeedforward<Volt>,
      kV: VelocityFeedforward<Radian, Volt>,
      kA: AccelerationFeedforward<Radian, Volt>,
   */
}