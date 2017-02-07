package kaike.reliable.location.main

import kaike.reliable.location.data.ReliableLocationParameter
import kaike.reliable.location.data.InstanceReader
import kaike.reliable.location.model.RUCFLSolver
import kaike.reliable.location.data.SolverInstructor
import kaike.reliable.location.output.NetworkOutput
import kaike.reliable.location.model.CrossMonmentSolver
import kaike.reliable.location.data.CrossMonmentParameter
import kaike.reliable.location.data.CrossMonmentProblemInstance

object CrossMonmentMain {
  def main(args: Array[String]): Unit = {
    val (demands, dcs) = InstanceReader.readInstanceFrom("input//UCFLData10.txt")
    val parameter = CrossMonmentParameter(alpha = 1.3, theta = 800)
    val instance = CrossMonmentProblemInstance(demands, dcs, parameter)
    val instructor = SolverInstructor()

    val model = new CrossMonmentSolver(instance, instructor)

    model.solve() match {
      case Some(sol) => NetworkOutput.post(sol)
      case _         => println("Unable to find solution.")
    }

  }
}