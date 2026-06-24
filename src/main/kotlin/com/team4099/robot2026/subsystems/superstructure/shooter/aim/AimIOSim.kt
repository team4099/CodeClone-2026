package com.team4099.robot2026.subsystems.superstructure.shooter.aim

import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.AimConstants
import com.team4099.robot2026.config.constants.Constants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team4099.lib.controller.PIDController
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Fraction
import org.team4099.lib.units.base.Second
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object AimIOSim: AimIO {
  private val aimSim: FlywheelSim = FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX44(1),
  AimConstants.MOMENT_OF_INERTIA.inKilogramsMeterSquared, 1.0/ AimConstants.GEAR_RATIO,), DCMotor.getKrakenX44(1))

  private val aimPIDController = PIDController(AimConstants.PID.SIM_KP, AimConstants.PID.SIM_KI, AimConstants.PID.SIM_KD)
  private var aimFFController = SimpleMotorFeedforward(AimConstants.PID.SIM_KS, AimConstants.PID.SIM_KV, AimConstants.PID.SIM_KA)

  override fun updateInputs(inputs: AimIO.AimInputs) {
    aimSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    //does ts add it to the loop cycle
    inputs.aimAcceleration = aimSim.angularAccelerationRadPerSecSq.radians.perSecond.perSecond
    inputs.aimVelocity = aimSim.angularVelocityRadPerSec.radians.perSecond
    inputs.aimAngle = 0.0.radians
    //i feel like there would be trig for aim angle i just can't think of it
    inputs.aimSupplyCurrent = 0.0.amps
    inputs.aimStatorCurrent = aimSim.currentDrawAmps.amps
    inputs.aimTorqueCurrent = aimSim.currentDrawAmps.amps.absoluteValue
    inputs.aimVoltage = aimSim.inputVoltage.volts
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage = clamp(voltage, -AimConstants.VOLTAGE_COMPENSATION, AimConstants.VOLTAGE_COMPENSATION)
    aimSim.setInputVoltage(clampedVoltage.inVolts)
  }

  override fun setVelocity(velocity: AngularVelocity) {
    var pidOutput  = aimPIDController.calculate(aimSim.angularVelocityRadPerSec.radians.perSecond, velocity)
    if (pidOutput.inVolts.isNaN()) pidOutput = 0.volts
    val ffOutput =
      aimFFController.calculateWithVelocities(aimSim.angularVelocityRadPerSec.radians.perSecond, velocity)
    setVoltage(pidOutput + ffOutput)
  }

  override fun configurePIDVoltage(
    kP: ProportionalGain<Fraction<Radian, Second>, Volt>,
    kI: IntegralGain<Fraction<Radian, Second>, Volt>,
    kD: DerivativeGain<Fraction<Radian, Second>, Volt>
  ) {
    aimPIDController.setPID(kP, kI, kD)
  }

  override fun configureFFVoltage(
    kS: StaticFeedforward<Volt>,
    kV: VelocityFeedforward<Radian, Volt>,
    kA: AccelerationFeedforward<Radian, Volt>
  ) {
    aimFFController = SimpleMotorFeedforward(kS, kV, kA)
  }

}
