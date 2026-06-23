package com.team4099.robot2026.subsystems.superstructure.indexer

import com.team4099.robot2026.subsystems.superstructure.Request.IndexerRequest
import com.team4099.robot2026.util.ControlledByStateMachine
import com.team4099.robot2026.util.CustomLogger
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

class Indexer(private val io: IndexerIO) : ControlledByStateMachine() {
  val inputs = IndexerIO.IndexerInputs()

  var targetVoltage = 0.0.volts
    private set

  var currentState: IndexerState = IndexerState.UNINITIALIZED
  var currentRequest: IndexerRequest = IndexerRequest.Idle()
    set(value) {
      when (value) {
        is IndexerRequest.OpenLoop -> targetVoltage = value.voltage
        else -> {}
      }
    }

  override fun onLoop() {
    io.updateInputs(inputs)
    CustomLogger.processInputs("Indexer", inputs)

    CustomLogger.recordOutput("Indexer/currentState", currentState)
    CustomLogger.recordOutput("Indexer/currentRequest", currentRequest.javaClass.simpleName)

    CustomLogger.recordOutput("Indexer/targetVoltageVolts", targetVoltage.inVolts)

    var nextState = currentState
    when (currentState) {
      IndexerState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      IndexerState.IDLE -> {
        nextState = fromRequestToState(currentRequest)
      }
      IndexerState.OPEN_LOOP -> {
        io.setVoltage(targetVoltage)
        nextState = fromRequestToState(currentRequest)
      }
    }
    currentState = nextState
  }

  companion object {
    enum class IndexerState {
      UNINITIALIZED,
      IDLE,
      OPEN_LOOP
    }

    inline fun fromRequestToState(request: IndexerRequest): IndexerState {
      return when (request) {
        is IndexerRequest.Idle -> IndexerState.IDLE
        is IndexerRequest.OpenLoop -> IndexerState.OPEN_LOOP
      }
    }
  }
}
