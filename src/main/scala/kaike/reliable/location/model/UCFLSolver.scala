package kaike.reliable.location.model

import kaike.reliable.location.data.ProblemInstance
import kaike.reliable.location.data.SolverInstructor
import kaike.reliable.location.data.LocationSolution
import ilog.cplex.IloCplex
import ilog.cplex.CpxException
import scala.util.control.NonFatal
import kaike.reliable.location.data.LocationSolution

class UCFLSolver(instance: ProblemInstance, instructor: SolverInstructor) extends Solver("UCFL"){
  val demands = instance.demandPoints
  val candidateDCs = instance.candidateLocations
  val distanceMap = instance.distanceMap

  def solve(): Option[LocationSolution] = {
    var ret: Option[LocationSolution] = None

    val cplex = new IloCplex()

    try {
      val open = candidateDCs.map { x => (x, cplex.boolVar()) }.toMap
      val assign = (for (demand <- demands; dc <- candidateDCs) yield ((demand, dc), cplex.boolVar())).toMap

      //      val objective = 
      val locationCosts = candidateDCs.map { dc => cplex.prod(dc.fixedCosts, open(dc)) }.fold(cplex.numExpr())(cplex.sum)
      val transportationCosts = (for (demand <- demands; dc <- candidateDCs)
        yield cplex.prod(demand.demand * distanceMap(demand, dc), assign(demand, dc))).fold(cplex.numExpr())(cplex.sum)

      val objective = cplex.sum(locationCosts, transportationCosts)
      cplex.addMinimize(objective)
      
      demands.foreach { demand => {
        val left = candidateDCs.map { dc => assign(demand, dc) }.fold(cplex.numExpr())(cplex.sum)
        cplex.addGe(left, 1)
      } }
      
      for(dc <- candidateDCs; demand <- demands) {
        cplex.addLe(assign(demand, dc), open(dc)) 
      }

      cplex.setParam(IloCplex.DoubleParam.TiLim, instructor.timeLimit)
      val begin = System.currentTimeMillis()
      if (cplex.solve()) {
        val end = System.currentTimeMillis()
        val openValues = candidateDCs.map { dc => (dc, cplex.getValue(open(dc))) }.toMap
        val openDCs = openValues.filter(p => p._2 > 0.5).keys.toSeq
        val assignments = (for (demand <- demands; dc <- candidateDCs)
          yield ((demand, dc), cplex.getValue(assign(demand, dc)))
        ).toMap.filter(p => p._2 > 0.5).keys.toSeq
        
        ret = Some(LocationSolution(instance = instance, openDCs = openDCs, assignments = assignments, 
            time = 1.0 * (end - begin) /1000, solver = this.SOLVER_NAME, objValue = cplex.getObjValue().toInt ))
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