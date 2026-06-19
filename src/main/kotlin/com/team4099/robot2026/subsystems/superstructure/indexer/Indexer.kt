package com.team4099.robot2026.subsystems.superstructure.indexer

import com.team4099.robot2026.config.constants.IndexerConstants
import com.team4099.robot2026.subsystems.superstructure.Request.IndexerRequest
import com.team4099.robot2026.util.ControlledByStateMachine
import com.team4099.robot2026.util.CustomLogger
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perMinute

class Indexer(private val io: IndexerIO) : ControlledByStateMachine() {

  companion object {
    enum class IndexerState {
      UNINITIALIZED,
      IDLE,
      OPEN_LOOP,
      TARGETING_VELOCITY
    }

    inline fun fromRequestToState(request: IndexerRequest): IndexerState {
      return when (request) {
        is IndexerRequest.Idle -> IndexerState.IDLE
        is IndexerRequest.OpenLoop -> IndexerState.OPEN_LOOP
        is IndexerRequest.TargetVelocity -> IndexerState.TARGETING_VELOCITY
      }
    }
  }

  val inputs = IndexerIO.IndexerInputs()

  var targetVoltage = 0.0.volts
    private set

  var targetVelocity = 0.0.rotations.perMinute
    private set

  var currentState: IndexerState = IndexerState.UNINITIALIZED
  var currentRequest: IndexerRequest = IndexerRequest.Idle()
    set(value) {
      when (value) {
        is IndexerRequest.OpenLoop -> targetVoltage = value.voltage
        is IndexerRequest.TargetVelocity -> targetVelocity = value.velocity
        else -> {}
      }
    }

  override fun onLoop() {
    io.updateInputs(inputs)
    CustomLogger.processInputs("Indexer", inputs)

    CustomLogger.recordOutput("Indexer/currentState", currentState)
    CustomLogger.recordOutput("Indexer/currentRequest", currentRequest.javaClass.simpleName)

    CustomLogger.recordOutput("Indexer/targetVoltage", targetVoltage.inVolts)
    CustomLogger.recordOutput("Indexer/targetVelocity", targetVelocity.inRotationsPerMinute)

    var nextState = currentState
    when (currentState) {
      IndexerState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      IndexerState.IDLE -> {
        io.setVelocity(IndexerConstants.VELOCITIES.IDLE_VELOCITY)
        nextState = fromRequestToState(currentRequest)
      }
      IndexerState.OPEN_LOOP -> {
        io.setVoltage(targetVoltage)
        nextState = fromRequestToState(currentRequest)
      }
      IndexerState.TARGETING_VELOCITY -> {
        io.setVelocity(targetVelocity)
        nextState = fromRequestToState(currentRequest)
      }
    }
    currentState = nextState
  }
}
