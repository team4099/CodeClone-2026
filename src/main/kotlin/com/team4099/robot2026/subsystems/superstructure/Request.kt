package com.team4099.robot2026.subsystems.superstructure

import org.team4099.lib.units.base.Length
import org.team4099.lib.units.derived.ElectricalPotential

sealed interface Request {
  sealed interface SuperstructureRequest : Request {}

  sealed interface IndexerRequest : Request {
    class Idle() : IndexerRequest

    class OpenLoop(val voltage: ElectricalPotential) : IndexerRequest
  }

  sealed interface FeederRequest : Request {
    class Idle() : FeederRequest

    class OpenLoop(val voltage: ElectricalPotential) : FeederRequest
  }

  sealed interface IntakeRollerRequest : Request {
    class Idle() : IntakeRollerRequest

    class OpenLoop(val voltage: ElectricalPotential) : IntakeRollerRequest
  }

  sealed interface LintakeRequest : Request {
    class Idle() : LintakeRequest

    class OpenLoop(val voltage: ElectricalPotential) : LintakeRequest

    class ClosedLoop(val position: Length) : LintakeRequest
  }
}
