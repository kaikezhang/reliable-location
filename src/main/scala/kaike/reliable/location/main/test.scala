package kaike.reliable.location.main

import kaike.reliable.location.data.InstanceReader
import kaike.reliable.location.model.UFLPSolver
import kaike.reliable.location.data.SolverInstructor
import kaike.reliable.location.model.UFLPSolver
import kaike.reliable.location.output.NetworkOutput
import kaike.reliable.location.model.RUFLPSolver
import kaike.reliable.location.data.RobustReliableLocationParameter
import kaike.reliable.location.data.StochasticReliableLocationProblemInstance
import kaike.reliable.location.model.RobustUFLPSolver
import kaike.reliable.location.data.RobustReliableLocationProblemInstance
import kaike.reliable.location.data.StochasticReliableLocationParameter

object test {
  def StocasticRUCLP() = {
    val (demands, dcs) = InstanceReader.readInstanceFrom("input//UCFLData50.txt")
    val parameter = StochasticReliableLocationParameter(alpha = 1 )
    val instance = StochasticReliableLocationProblemInstance(demands, dcs, parameter)
    val instructor =  SolverInstructor()

    val model = new RUFLPSolver(instance, instructor)
    
    model.solve() match {
      case Some(sol) => NetworkOutput.post(sol)
      case _ => println("Unable to find solution.")
    }    
  }
  
  def RobustRUCLP() = {
    val (demands, dcs) = InstanceReader.readInstanceFrom("input//UCFLData50.txt")
    val parameter = RobustReliableLocationParameter(alpha = 1 )
    val instance = RobustReliableLocationProblemInstance(demands, dcs, parameter)
    val instructor =  SolverInstructor()

    val model = new RobustUFLPSolver(instance, instructor)
    
    model.solve() match {
      case Some(sol) => NetworkOutput.post(sol)
      case _ => println("Unable to find solution.")
    }      
  }
  
  
  def main(args: Array[String]): Unit = {
    RobustRUCLP()
  }
}