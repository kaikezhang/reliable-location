package kaike.reliable.location.main

import kaike.reliable.location.data.ReliableLocationParameter
import kaike.reliable.location.data.InstanceReader
import kaike.reliable.location.model.RUFLPSolver
import kaike.reliable.location.data.SolverInstructor
import kaike.reliable.location.data.StochasticReliableLocationProblemInstance
import kaike.reliable.location.output.NetworkOutput
import kaike.reliable.location.model.RobustUFLPSolver
import kaike.reliable.location.data.RobustReliableLocationProblemInstance

object RobustMain {
  def main(args: Array[String]): Unit = {
    val (demands, dcs) = InstanceReader.readInstanceFrom("input//UCFLData50.txt")
    val parameter = ReliableLocationParameter(alpha = 0.3, theta = 200)
    val instance = RobustReliableLocationProblemInstance(demands, dcs, parameter)
    val instructor =  SolverInstructor()

    val model = new RobustUFLPSolver(instance, instructor)
    
    model.solve() match {
      case Some(sol) => NetworkOutput.post(sol)
      case _ => println("Unable to find solution.")
    }
  }
}