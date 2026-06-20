package com.team4099.robot2026.subsystems.superstructure.shooter

import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.ShooterConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team4099.lib.controller.PIDController
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Fraction
import org.team4099.lib.units.base.Second
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.AccelerationFeedforward
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

object FlywheelIOSim : FlywheelIO {
    private val flywheelSim: FlywheelSim =
        FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(2),
                ShooterConstants.MOMENT_OF_INERTIA.inKilogramsMeterSquared,
                1.0 / ShooterConstants.GEAR_RATIO,
            ),
            DCMotor.getKrakenX60(2))

    private val flywheelPIDController =
        PIDController(
            ShooterConstants.PID.SIM_KP, ShooterConstants.PID.SIM_KI, ShooterConstants.PID.SIM_KD)

    private var flywheelFFController =
        SimpleMotorFeedforward(
            ShooterConstants.PID.SIM_KS, ShooterConstants.PID.SIM_KV, ShooterConstants.PID.SIM_KA)

    override fun updateInputs(inputs: FlywheelIO.FlywheelInputs) {
        flywheelSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
        inputs.flywheelLeaderVelocity = flywheelSim.angularVelocityRadPerSec.radians.perSecond
        inputs.flywheelLeaderAcceleration = flywheelSim.angularAccelerationRadPerSecSq.radians.perSecond.perSecond
        inputs.flywheelLeaderVoltage = flywheelSim.inputVoltage.volts
        inputs.flywheelLeaderSupplyCurrent = 0.0.amps
        inputs.flywheelLeaderStatorCurrent = flywheelSim.currentDrawAmps.amps
        inputs.flywheelLeaderTorqueCurrent = flywheelSim.currentDrawAmps.amps.absoluteValue
        inputs.flywheelLeaderTemperature = 0.0.celsius

        inputs.flywheelFollowerVelocity = flywheelSim.angularVelocityRadPerSec.radians.perSecond
        inputs.flywheelFollowerAcceleration = flywheelSim.angularAccelerationRadPerSecSq.radians.perSecond.perSecond
        inputs.flywheelFollowerVoltage = flywheelSim.inputVoltage.volts
        inputs.flywheelFollowerSupplyCurrent = 0.0.amps
        inputs.flywheelFollowerStatorCurrent = flywheelSim.currentDrawAmps.amps
        inputs.flywheelFollowerTorqueCurrent = flywheelSim.currentDrawAmps.amps.absoluteValue
        inputs.flywheelFollowerTemperature = 0.0.celsius
    }

    override fun setVoltage(voltage: ElectricalPotential) {
        val clampedVoltage =
            clamp(
                voltage, -ShooterConstants.VOLTAGE_COMPENSATION, ShooterConstants.VOLTAGE_COMPENSATION)
        flywheelSim.setInputVoltage(clampedVoltage.inVolts)
    }

    override fun setVelocity(velocity: AngularVelocity) {
        var pidOutput = flywheelPIDController.calculate(flywheelSim.angularVelocityRadPerSec.radians.perSecond, velocity)
        if (pidOutput.inVolts.isNaN()) pidOutput = 0.volts
        val ffOutput = flywheelFFController.calculateWithVelocities(flywheelSim.angularVelocityRadPerSec.radians.perSecond, velocity)
        setVoltage(pidOutput + ffOutput)
    }

    override fun configurePIDVoltage(
        kP: ProportionalGain<Fraction<Radian, Second>, Volt>,
        kI: IntegralGain<Fraction<Radian, Second>, Volt>,
        kD: DerivativeGain<Fraction<Radian, Second>, Volt>
    ) {
        flywheelPIDController.setPID(kP, kI, kD)
    }

    override fun configureFFVoltage(
        kS: StaticFeedforward<Volt>,
        kV: VelocityFeedforward<Radian, Volt>,
        kA: AccelerationFeedforward<Radian, Volt>
    ) {
        flywheelFFController = SimpleMotorFeedforward(kS, kV, kA)
    }
}



