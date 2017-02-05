package kaike.reliable.location.model

import kaike.reliable.location.data.ReliableLocationProblemInstance
import kaike.reliable.location.data.SolverInstructor
import kaike.reliable.location.data.LocationSolution
import ilog.cplex.IloCplex
import ilog.cplex.CpxException
import scala.util.control.NonFatal
import kaike.reliable.location.data.LocationSolution

class UCFLSolver(instance: ReliableLocationProblemInstance, instructor: SolverInstructor) extends Solver("UCFL"){
  val demands = instance.demandPoints
  val candidateDCs = instance.candidateLocations
  
  val demandIndexes = instance.demandsPointIndexes
  val locationIndexes = instance.candidateLocationIndexes
  
  val distance = instance.distance

  def solve(): Option[LocationSolution] = {
    var ret: Option[LocationSolution] = None

    val cplex = new IloCplex()

    try {
      val open = Array.tabulate(candidateDCs.size)( i => cplex.boolVar() )
      val assign = Array.tabulate(demands.size, candidateDCs.size)((i, j) => {cplex.boolVar()})

      //      val objective = 
      val locationCosts = locationIndexes.map { j => cplex.prod(candidateDCs(j).fixedCosts, open(j)) }.fold(cplex.numExpr())(cplex.sum)
      
      val transportationCosts = (for (i <- demandIndexes; j <- locationIndexes)
        yield cplex.prod(demands(i).demand * distance(i)(j), assign(i)(j))).fold(cplex.numExpr())(cplex.sum)

      val objective = cplex.sum(locationCosts, transportationCosts)
      cplex.addMinimize(objective)
      
      demandIndexes.foreach { i => {
        val left = locationIndexes.map { j => assign(i)(j) }.fold(cplex.numExpr())(cplex.sum)
        cplex.addGe(left, 1)
      } }
      
      for(i <- demandIndexes; j <- locationIndexes ) {
        cplex.addLe(assign(i)(j), open(j)) 
      }

      cplex.setParam(IloCplex.DoubleParam.TiLim, instructor.timeLimit)
      val begin = System.currentTimeMillis()
      if (cplex.solve()) {
        val end = System.currentTimeMillis()
        val openValues = locationIndexes.map { j => (candidateDCs(j), cplex.getValue(open(j))) }.toMap
        val openDCs = openValues.filter(p => p._2 > 0.5).keys.toSeq
        val assignments = (for (i <- demandIndexes; j <- locationIndexes)
          yield ((demands(i), candidateDCs(j)), cplex.getValue(assign(i)(j)))
        ).toMap.filter(p => p._2 > 0.5).keys.toSeq
        
        ret = Some(LocationSolution(instance = instance, openDCs = openDCs, assignments = assignments, 
            time = 1.0 * (end - begin) /1000, solver = this, objValue = Math.round(cplex.getObjValue()), gap = cplex.getMIPRelativeGap ))
      }

    } catch {
      case e: CpxException => println("Cplex exception caught: " + e);
      case NonFatal(e)     => println("exception caught: " + e);
      case _: Throwable    =>
    } finally {
      cplex.end()
    }
    ret
  }
}