package kaike.reliable.location.main

import kaike.reliable.location.data.ReliableLocationParameter
import kaike.reliable.location.data.InstanceReader
import kaike.reliable.location.model.RUCFLSolver
import kaike.reliable.location.data.SolverInstructor
import kaike.reliable.location.data.ReliableLocationProblemInstance
import kaike.reliable.location.output.Visualizer
import kaike.reliable.location.model.CrossMonmentSolver
import kaike.reliable.location.data.CrossMonmentParameter
import kaike.reliable.location.data.CrossMonmentProblemInstance

object CrossMonmentMain {
  def main(args: Array[String]): Unit = {
    val (demands, dcs) = InstanceReader.readInstanceFrom("input//UCFLData20.txt")
    val parameter = CrossMonmentParameter(beta = 0.1, theta = 400)
    val instance = CrossMonmentProblemInstance(demands, dcs, parameter)
    val instructor = SolverInstructor(gap = 0.000001)

    val model = new CrossMonmentSolver(instance, instructor)

    model.solve() match {
      case Some(sol) => Visualizer.post(sol)
      case _         => println("Unable to find solution.")
    }

  }
}