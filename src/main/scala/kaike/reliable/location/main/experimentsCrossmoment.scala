package kaike.reliable.location.main

import kaike.reliable.location.data.CrossMomentParameter
import kaike.reliable.location.data.CrossMomentProblemInstance
import kaike.reliable.location.data.InstanceReader
import kaike.reliable.location.output.NetworkOutput
import kaike.reliable.location.model.RUFLPSolver
import kaike.reliable.location.data.SolverInstructor
import kaike.reliable.location.model.CrossMomentSolver
import java.io.ByteArrayOutputStream

object experimentsCrossmoment {
  
  def main(args: Array[String]): Unit = {

    val nodes = List( "10", "20", "30")
    val alphas = (1.0 to 1.5 by 0.1)
    val thetas = List( 200, 400, 800)

    val matrixType = 5
    
    for (node <- nodes; alpha <- alphas; theta <- thetas) {
      val outCapture = new ByteArrayOutputStream
      Console.withOut(outCapture) {
        val (demands, dcs) = InstanceReader.readInstanceFrom(s"input//UCFLData${node}.txt")
        val parameter = CrossMomentParameter(alpha = alpha, theta = theta, matrixType = matrixType)
        val instance = CrossMomentProblemInstance(demands, dcs, parameter)
        val instructor = SolverInstructor()
        val model = new CrossMomentSolver(instance, instructor)

        model.solve() match {
          case Some(sol) => NetworkOutput.post(sol, trim(outCapture.toString()))
          case _         => NetworkOutput.postError(model, trim(outCapture.toString()))
        }
      }
      outCapture.close()
    }

  }
}