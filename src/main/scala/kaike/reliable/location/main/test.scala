package kaike.reliable.location.main

import kaike.reliable.location.data.InstanceReader
import kaike.reliable.location.model.UFLPSolver
import kaike.reliable.location.data.SolverInstructor
import kaike.reliable.location.model.UFLPSolver
import kaike.reliable.location.output.Visualizer

object test {
  def main(args: Array[String]): Unit = {
    val instance = InstanceReader.readInstanceFrom("input//UCFLData100.txt")
    val instructor =  SolverInstructor()
    val model = new UFLPSolver(instance, instructor)
    model.solve() match {
      case Some(sol) => Visualizer.post(sol)
      case _ => println("Unable to find solution.")
    }
    
  }
}