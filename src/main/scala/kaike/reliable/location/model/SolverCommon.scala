package kaike.reliable.location.model

import kaike.reliable.location.data.LocationSolution
import kaike.reliable.location.data.ProblemInstance
import kaike.reliable.location.data.SolverInstructor


abstract class SolverCommon(val instance:ProblemInstance, val instructor:SolverInstructor, val SOLVER_NAME:String) {
  def solve():Option[LocationSolution]
  
  val demands = instance.demandPoints
  val candidateDCs = instance.candidateLocations
  
  val demandIndexes = instance.demandsPointIndexes
  val locationIndexes = instance.candidateLocationIndexes
  
  val distance = instance.distance  
  
  var beginTime: Double = _
  var openDCs = Seq.empty[Int]

  def timeUsed(): Double = {
    1.0 * (System.currentTimeMillis() - beginTime) / 1000
  }
  def timeLeft(): Double = {
    instructor.timeLimit - timeUsed
  }

  def timeLimitReached(): Boolean = {
    timeLeft < 0
  }
  
}