package kaike.reliable.location.main

import kaike.reliable.location.data.RobustReliableLocationParameter
import kaike.reliable.location.data.InstanceReader
import kaike.reliable.location.output.NetworkOutput
import kaike.reliable.location.model.RUFLPSolver
import kaike.reliable.location.data.SolverInstructor
import kaike.reliable.location.data.StochasticReliableLocationProblemInstance
import kaike.reliable.location.data.StochasticReliableLocationParameter
import java.io.ByteArrayOutputStream
import kaike.reliable.location.model.RUFLPSimpleCutSolver


object experimentsStochasticRUFLP {
  
  def main(args: Array[String]): Unit = {
    
    val nodes = List("50", "75", "100")
    val alphas = (1.0 to 1.5 by 0.05)
    
    for(node <- nodes; alpha <- alphas){
      val outCapture = new ByteArrayOutputStream
      Console.withOut(outCapture) {
        val (demands, dcs) = InstanceReader.readInstanceFrom(s"input//UCFLData${node}.txt")
        val parameter = StochasticReliableLocationParameter(alpha = alpha )
        val instance = StochasticReliableLocationProblemInstance(demands, dcs, parameter)
        val instructor =  SolverInstructor(gap = 0.000001)
    
        val model = new RUFLPSolver(instance, instructor)
//        val model = new RUFLPSimpleCutSolver(instance, instructor)
        
        model.solve() match {
          case Some(sol) => Console.out.flush(); NetworkOutput.post(sol, trim(outCapture.toString()))
          case _ => //NetworkOutput.postError(model)
        }
        outCapture.close()
    }      
    }

    
  }
}