package kaike.reliable.location.main

import kaike.reliable.location.data.RobustReliableLocationParameter
import kaike.reliable.location.data.InstanceReader
import kaike.reliable.location.model.RUFLPSolver
import kaike.reliable.location.data.SolverInstructor
import kaike.reliable.location.output.NetworkOutput
import kaike.reliable.location.model.CrossMomentSolver
import kaike.reliable.location.data.CrossMomentParameter
import kaike.reliable.location.data.CrossMomentProblemInstance
import java.io.PrintStream
import java.io.ByteArrayOutputStream
import kaike.reliable.location.data.LocationSolution

object CrossMomentMain {
  def main(args: Array[String]): Unit = {
    val outCapture = new ByteArrayOutputStream
    
    Console.withOut(outCapture) {
        val (demands, dcs) = InstanceReader.readInstanceFrom("input//UCFLData10.txt")
        val parameter = CrossMomentParameter(alpha = 1.0, theta = 100)
        val instance = CrossMomentProblemInstance(demands, dcs, parameter)
        val instructor = SolverInstructor()

        val model = new CrossMomentSolver(instance, instructor)
        
        model.solve() match {
              case Some(sol) => NetworkOutput.post(sol, trim(outCapture.toString()))
              case _         => NetworkOutput.postError(model, trim(outCapture.toString()))
        }        
    }
    

    
  }
}