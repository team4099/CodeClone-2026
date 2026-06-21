package com.team4099.robot2026.subsystems.superstructure.intake.rollers

import com.team4099.robot2026.subsystems.superstructure.Request
import com.team4099.robot2026.util.ControlledByStateMachine
import com.team4099.robot2026.util.CustomLogger
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

class IntakeRollers(private val io: IntakeRollersIO) : ControlledByStateMachine() {
  var targetVoltage = 0.0.volts
    private set

  val inputs = IntakeRollersIO.RollerInputs()
  var currentState = IntakeRollersState.UNINITIALIZED
  var currentRequest: Request.IntakeRollerRequest = Request.IntakeRollerRequest.Idle()
    set(value) {
      when (value) {
        is Request.IntakeRollerRequest.OpenLoop -> targetVoltage = value.voltage
        is Request.IntakeRollerRequest.Idle ->
            targetVoltage = IntakeConstants.RollerIntakeConstants.IDLE_VOLTAGE
      }
      field = value
    }

  override fun onLoop() {
    io.updateInputs(inputs)
    CustomLogger.processInputs("Intake Rollers", inputs)
    CustomLogger.recordOutput("IntakeRollers/CurrentState", currentState.name)
    CustomLogger.recordOutput("IntakeRollers/CurrentRequest", currentRequest.javaClass.simpleName)
    CustomLogger.recordOutput("IntakeRollers/TargetVoltage", targetVoltage.inVolts)
    var nextState = currentState
    when (currentState) {
      IntakeRollersState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)
      }
      IntakeRollersState.OPEN_LOOP -> {
        io.setVoltage(targetVoltage)
        nextState = fromRequestToState(currentRequest)
      }
      IntakeRollersState.IDLE -> {
        io.setVoltage(IntakeConstants.RollerIntakeConstants.IDLE_VOLTAGE)
        nextState = fromRequestToState(currentRequest)
      }
    }
    currentState = nextState
  }

  companion object {
    enum class IntakeRollersState {
      OPEN_LOOP,
      IDLE,
      UNINITIALIZED
    }

    inline fun fromRequestToState(request: Request.IntakeRollerRequest): IntakeRollersState {
      return when (request) {
        is Request.IntakeRollerRequest.Idle -> IntakeRollersState.IDLE
        is Request.IntakeRollerRequest.OpenLoop -> IntakeRollersState.OPEN_LOOP
      }
    }
  }
}
