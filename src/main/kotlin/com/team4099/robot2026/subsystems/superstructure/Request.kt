package com.team4099.robot2026.subsystems.superstructure

import org.team4099.lib.units.AngularVelocity
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
}
