package com.team4099.robot2026.subsystems.superstructure.feeder

import com.team4099.robot2026.config.constants.FeederConstants
import com.team4099.robot2026.subsystems.superstructure.Request.FeederRequest
import com.team4099.robot2026.util.ControlledByStateMachine
import com.team4099.robot2026.util.CustomLogger
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

class Feeder(private val io: FeederIO) : ControlledByStateMachine() {
  val inputs = FeederIO.FeederInputs()

  var targetVoltage = 0.0.volts
    private set

  var currentState: FeederState = FeederState.UNINITIALIZED
  var currentRequest: FeederRequest = FeederRequest.Idle()
    set(value) {
      when (value) {
        is FeederRequest.OpenLoop -> targetVoltage = value.voltage
        else -> {}
      }
    }

  override fun onLoop() {
    io.updateInputs(inputs)
    CustomLogger.processInputs("Feeder", inputs)

    CustomLogger.recordOutput("Feeder/currentState", currentState)
    CustomLogger.recordOutput("Feeder/currentRequest", currentRequest.javaClass.simpleName)

    CustomLogger.recordOutput("Feeder/targetVoltageVolts", targetVoltage.inVolts)

    var nextState = currentState
    when (currentState) {
      FeederState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      FeederState.IDLE -> {
        io.setVelocity(FeederConstants.VELOCITIES.IDLE_VELOCITY)
        nextState = fromRequestToState(currentRequest)
      }
      FeederState.OPEN_LOOP -> {
        io.setVoltage(targetVoltage)
        nextState = fromRequestToState(currentRequest)
      }
    }
    currentState = nextState
  }

  companion object {
    enum class FeederState {
      UNINITIALIZED,
      IDLE,
      OPEN_LOOP
    }

    inline fun fromRequestToState(request: FeederRequest): FeederState {
      return when (request) {
        is FeederRequest.Idle -> FeederState.IDLE
        is FeederRequest.OpenLoop -> FeederState.OPEN_LOOP
      }
    }
  }
}