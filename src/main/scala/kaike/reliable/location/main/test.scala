package kaike.reliable.location.main

import kaike.reliable.location.data.InstanceReader
import kaike.reliable.location.model.UCFLSolver
import kaike.reliable.location.data.SolverInstructor
import kaike.reliable.location.model.UCFLSolver
import kaike.reliable.location.output.NetworkOutput
import kaike.reliable.location.model.RUCFLSolver
import kaike.reliable.location.data.ReliableLocationParameter
import kaike.reliable.location.data.StochasticReliableLocationProblemInstance
import kaike.reliable.location.model.RobustUCFLSolver
import kaike.reliable.location.data.RobustReliableLocationProblemInstance

object test {
  def StocasticRUCLP() = {
    val (demands, dcs) = InstanceReader.readInstanceFrom("input//UCFLData50.txt")
    val parameter = ReliableLocationParameter(alpha = 1 )
    val instance = StochasticReliableLocationProblemInstance(demands, dcs, parameter)
    val instructor =  SolverInstructor()

    val model = new RUCFLSolver(instance, instructor)
    
    model.solve() match {
      case Some(sol) => NetworkOutput.post(sol)
      case _ => println("Unable to find solution.")
    }    
  }
  
  def RobustRUCLP() = {
    val (demands, dcs) = InstanceReader.readInstanceFrom("input//UCFLData50.txt")
    val parameter = ReliableLocationParameter(alpha = 1 )
    val instance = RobustReliableLocationProblemInstance(demands, dcs, parameter)
    val instructor =  SolverInstructor()

    val model = new RobustUCFLSolver(instance, instructor)
    
    model.solve() match {
      case Some(sol) => NetworkOutput.post(sol)
      case _ => println("Unable to find solution.")
    }      
  }
  
  
  def main(args: Array[String]): Unit = {
    RobustRUCLP()
  }
}