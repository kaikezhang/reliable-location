package kaike.reliable.location.main

import kaike.reliable.location.data.ReliableLocationParameter
import kaike.reliable.location.data.InstanceReader
import kaike.reliable.location.model.RUCFLSolver
import kaike.reliable.location.data.SolverInstructor
import kaike.reliable.location.data.ReliableLocationProblemInstance
import kaike.reliable.location.output.Visualizer
import kaike.reliable.location.model.RobustUCFLSolver
import kaike.reliable.location.data.RobustLocationProblemInstance

object RobustMain {
  def main(args: Array[String]): Unit = {
    val (demands, dcs) = InstanceReader.readInstanceFrom("input//UCFLData20.txt")
    val parameter = ReliableLocationParameter(alpha = 0.1, theta = 400)
    val instance = RobustLocationProblemInstance(demands, dcs, parameter)
    val instructor =  SolverInstructor()

    val model = new RobustUCFLSolver(instance, instructor)
    
    model.solve() match {
      case Some(sol) => Visualizer.post(sol)
      case _ => println("Unable to find solution.")
    }
  }
}