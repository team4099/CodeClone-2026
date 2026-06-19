package com.team4099.robot2026.subsystems.superstructure.intake

import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.Constants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import org.team4099.lib.controller.ProfiledPIDController
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inGrams
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object LinearIntakeIOSim : LinearIntakeIO {
  private val lintakeSim: ElevatorSim =
      ElevatorSim(
          DCMotor.getKrakenX60(1),
          1.0 / IntakeConstants.LinearIntakeConstants.GEAR_RATIO,
          IntakeConstants.LinearIntakeConstants.LINTAKE_MASS.inGrams,
          IntakeConstants.LinearIntakeConstants.DIAMETER.inMeters,
          IntakeConstants.LintakePosConstants.FORWARD_EXTENSION_LIM.inMeters,
          IntakeConstants.LintakePosConstants.BACKMOST_EXTENSION_LIM.inMeters,
          false,
          IntakeConstants.LintakePosConstants.START_POSITION.inMeters)

  private var lastAppliedVoltage = 0.0.volts
  private var lintakePIDController =
      ProfiledPIDController(
          IntakeConstants.LintakePID.SIM_KP,
          IntakeConstants.LintakePID.SIM_KI,
          IntakeConstants.LintakePID.SIM_KD,
          TrapezoidProfile.Constraints(
              IntakeConstants.LinearIntakeConstants.MAX_VELOCITY,
              IntakeConstants.LinearIntakeConstants.MAX_ACCELERATION,
          ))

  override fun updateInputs(inputs: LinearIntakeIO.LintakeIOInputs) {
    lintakeSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    inputs.lintakePosition = lintakeSim.positionMeters.meters
    inputs.lintakeTemperature = 0.0.celsius
    inputs.lintakeStatorCurrent = lintakeSim.currentDrawAmps.amps
    inputs.lintakeSupplyCurrent = 0.0.amps
    inputs.lintakeAppliedVoltage = lastAppliedVoltage
    inputs.lintakeVelocity = lintakeSim.velocityMetersPerSecond.meters.perSecond
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(lintakeSim.currentDrawAmps))
  }

  override fun setVoltage(targetVoltage: ElectricalPotential) {
    val clampedVoltage =
        clamp(
            targetVoltage,
            -IntakeConstants.LinearIntakeConstants.VOLTAGE_COMPENSATION,
            IntakeConstants.LinearIntakeConstants.VOLTAGE_COMPENSATION)
    lastAppliedVoltage = clampedVoltage
    lintakeSim.setInputVoltage(clampedVoltage.inVolts)
  }

  override fun setPosition(position: Length) {
    lintakePIDController.setGoal(position)
    val pidOutput = lintakePIDController.calculate(lintakeSim.positionMeters.meters)
    setVoltage(pidOutput)
  }

  override fun zeroEncoder() {
    lintakeSim.setState(0.0, 0.0)
  }

  override fun configPID(
      kP: ProportionalGain<Meter, Volt>,
      kI: IntegralGain<Meter, Volt>,
      kD: DerivativeGain<Meter, Volt>
  ) {
    lintakePIDController.setPID(kP, kI, kD)
  }
}
