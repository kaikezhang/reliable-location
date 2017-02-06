package kaike.reliable.location.main

import kaike.reliable.location.data.InstanceReader
import kaike.reliable.location.model.UCFLSolver
import kaike.reliable.location.data.SolverInstructor
import kaike.reliable.location.model.UCFLSolver
import kaike.reliable.location.output.NetworkOutput
import kaike.reliable.location.model.RUCFLSolver
import kaike.reliable.location.data.ReliableLocationParameter
import kaike.reliable.location.data.ReliableLocationProblemInstance

object test {
  
  def main(args: Array[String]): Unit = {
    val (demands, dcs) = InstanceReader.readInstanceFrom("input//UCFLData50.txt")
    val parameter = ReliableLocationParameter(alpha = 1 )
    val instance = ReliableLocationProblemInstance(demands, dcs, parameter)
    val instructor =  SolverInstructor()

    val model = new RUCFLSolver(instance, instructor)
//    val model = new UCFLSolver(instance, instructor)
    
    model.solve() match {
      case Some(sol) => NetworkOutput.post(sol)
      case _ => println("Unable to find solution.")
    }
    
  }
}