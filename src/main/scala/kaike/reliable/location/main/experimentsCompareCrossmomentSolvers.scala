package kaike.reliable.location.main

import java.io.ByteArrayOutputStream
import kaike.reliable.location.data.CrossMomentParameter
import kaike.reliable.location.data.InstanceReader
import kaike.reliable.location.output.NetworkOutput
import kaike.reliable.location.data.CrossMomentProblemInstance
import kaike.reliable.location.data.SolverInstructor
import kaike.reliable.location.model.CrossMomentSolver
import kaike.reliable.location.model.CrossMomentTwoPhaseSolver
import kaike.reliable.location.model.CrossMomentSolverAbstract
import kaike.reliable.location.model.CrossMomentTwoPhaseSolver
import kaike.reliable.location.model.CrossMomentSolver
import kaike.reliable.location.model.CrossMomentTwoPhaseSolver
import kaike.reliable.location.model.CrossMomentSolver
import kaike.reliable.location.model.CrossMomentTwoPhaseSolver
import kaike.reliable.location.model.CrossMomentSolver

object experimentsCompareCrossmomentSolvers {
  def solveProblem(node:String, alpha:Double, theta:Int, matrixType:Int, failrateType:Int, solver:Class[_ <: CrossMomentSolverAbstract]) = {
      val outCapture = new ByteArrayOutputStream
      Console.withOut(outCapture) {
        val (demands, dcs) = InstanceReader.readInstanceFrom(s"input//UCFLData${node}.txt")
        val parameter = CrossMomentParameter(alpha = alpha, theta = theta, matrixType = matrixType, failrateType = failrateType)
        val instance = CrossMomentProblemInstance(demands, dcs, parameter)
        val instructor = SolverInstructor()
        val model = solver.getConstructors()(0).newInstance(instance, instructor).asInstanceOf[CrossMomentSolverAbstract]

        model.solve() match {
          case Some(sol) => NetworkOutput.post(sol, trim(outCapture.toString()))
          case _         => NetworkOutput.postError(model, trim(outCapture.toString()))
        }
      }
      outCapture.close()    
  }
  

  
  def main(args: Array[String]): Unit = {

    val nodes = List( "20", "30")
    val alphas = (0.1 to 0.31 by 0.1)
    val thetas = List( 100, 200, 400)

    val matrixType = 7
    val failrateType = 2
    
    for (node <- nodes; alpha <- alphas; theta <- thetas) {
      solveProblem(node, alpha, theta, matrixType, failrateType, classOf[CrossMomentSolver])
      solveProblem(node, alpha, theta, matrixType, failrateType, classOf[CrossMomentTwoPhaseSolver] )
    }

  }  
}