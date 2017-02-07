package kaike.reliable.location.main

import kaike.reliable.location.data.CrossMonmentParameter
import kaike.reliable.location.data.CrossMonmentProblemInstance
import kaike.reliable.location.data.InstanceReader
import kaike.reliable.location.output.NetworkOutput
import kaike.reliable.location.model.RUCFLSolver
import kaike.reliable.location.data.SolverInstructor
import kaike.reliable.location.model.CrossMonmentSolver


object experimentsCrossmonment {
  def main(args: Array[String]): Unit = {
    
    val nodes = List("10", "20", "50")
    val alphas = (1.0 to 1.5 by 0.05)
    val thetas = List(100, 200, 400)
    
    for(node <- nodes; alpha <- alphas; theta <- thetas){
      val (demands, dcs) = InstanceReader.readInstanceFrom(s"input//UCFLData${node}.txt")
      val parameter = CrossMonmentParameter(alpha = alpha, theta = theta)
      val instance = CrossMonmentProblemInstance(demands, dcs, parameter)
      val instructor = SolverInstructor()
      val model = new CrossMonmentSolver(instance, instructor)
      
      model.solve() match {
        case Some(sol) => NetworkOutput.post(sol)
        case _ => NetworkOutput.postNoSolutionFound(model)
      }      
    }

    
  }  
}