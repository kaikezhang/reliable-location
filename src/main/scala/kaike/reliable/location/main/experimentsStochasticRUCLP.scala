package kaike.reliable.location.main

import kaike.reliable.location.data.ReliableLocationParameter
import kaike.reliable.location.data.InstanceReader
import kaike.reliable.location.output.NetworkOutput
import kaike.reliable.location.model.RUCFLSolver
import kaike.reliable.location.data.SolverInstructor
import kaike.reliable.location.data.ReliableLocationProblemInstance


object experimentsStochasticRUCLP {
  
  def main(args: Array[String]): Unit = {
    
    val nodes = List("50", "75", "100", "150")
    val alphas = (1.0 to 1.5 by 0.05)
    
    for(node <- nodes; alpha <- alphas){
      val (demands, dcs) = InstanceReader.readInstanceFrom(s"input//UCFLData${node}.txt")
      val parameter = ReliableLocationParameter(alpha = alpha )
      val instance = ReliableLocationProblemInstance(demands, dcs, parameter)
      val instructor =  SolverInstructor()
  
      val model = new RUCFLSolver(instance, instructor)
      
      model.solve() match {
        case Some(sol) => NetworkOutput.post(sol)
        case _ => println("Unable to find solution.")
      }      
    }

    
  }
}