package kaike.reliable.location.model

import kaike.reliable.location.data.CrossMomentProblemInstance
import kaike.reliable.location.data.SolverInstructor
import kaike.reliable.location.data.Scenario

abstract class CrossMomentSolverAbstract(override val instance: CrossMomentProblemInstance, override val instructor: SolverInstructor, name:String) extends SolverCommon(instance, instructor, name)  {
  val SCALE_FACTOR = 1E12
  
  val crossMomentMatrix = instance.crossMomentMatrix.map { x => x.map { x => if(x > 0 ) x * SCALE_FACTOR  else x} }

  println(s"Cross moment matrix with scale factor = ${SCALE_FACTOR}:")
  crossMomentMatrix.foreach { x => println(x.map { x => x.toString.padTo(25, " ").mkString }.mkString("[", ", ", "]")) }
  println()

  val ArtificialSeedRealizations = instance.realizations
  
  var modelInfeasible = false

  var nbCuts = 0

  var upperBound = Double.MaxValue
  var lowerBound = 0.0

  var InitialScenarios = Array.empty[Scenario]  
}