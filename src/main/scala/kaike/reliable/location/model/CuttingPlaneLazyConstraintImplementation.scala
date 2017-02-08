package kaike.reliable.location.model

import ilog.cplex.IloCplex
import ilog.concert.IloIntVar
import ilog.concert.IloNumVar
import kaike.reliable.location.data.LocationSolution
import ilog.cplex.CpxException
import ilog.cplex.IloCplex.LazyConstraintCallback
import kaike.reliable.location.data.SolverInstructor
import kaike.reliable.location.data.StochasticReliableLocationProblemInstance
import scala.util.control.NonFatal
import scala.collection.immutable.TreeSet
import kaike.reliable.location.data.LocationSolution
import kaike.reliable.location.data.SolverInstructor
import kaike.reliable.location.data.StochasticReliableLocationProblemInstance
import kaike.reliable.location.data.ProblemInstance
import kaike.reliable.location.data.ReliableProblemInstance


abstract class CuttingPlaneLazyConstraintImplementation(val instance: ReliableProblemInstance,
                                                        val instructor: SolverInstructor,
                                                        val name:String) extends Solver(name){
  val demands = instance.demandPoints
  val candidateDCs = instance.candidateLocations
  
  val distance = instance.distance
  val failrate = instance.failRate
  
  val demandIndexes = instance.demandsPointIndexes
  val locationIndexes = instance.candidateLocationIndexes
  
  var nbCuts = 0
  
  var upperBound = Double.MaxValue
  var lowerBound = 0.0
  
  
  def newLazyCutClass(cplex: IloCplex, open: IndexedSeq[IloIntVar], phi: IloNumVar): LazyConstraintCallback
  
  

  class SuperModularCutLazyConstraint(cplex: IloCplex, open: IndexedSeq[IloIntVar], phi: IloNumVar) extends LazyConstraintCallback {
    
    // Need to be override by subclass
    def getTransptCosts(openLocs:Set[Int]):Double = {
      0.0
    }
    
    def main(): Unit = {
      val openValues = open.map { x => getValue(x) }
      val setupCosts = locationIndexes.map { j => openValues(j) * candidateDCs(j).fixedCosts }.sum
      
      val openLocs = locationIndexes.filter(j => openValues(j) > 0.5).toSet 

      
      val solutionTrspCosts = getTransptCosts(openLocs)


      val phiValue = getValue(phi)
      
      val clb = getBestObjValue()
      val cub = setupCosts + solutionTrspCosts
      
      if (lowerBound < clb) 
        lowerBound = clb

      if (upperBound > cub) 
        upperBound = cub

      if (lowerBound > 0) {
        if (((upperBound - lowerBound) / lowerBound) < instructor.gap) {
          println("No cut is added due to gap limit reached.")
          abort()
          return 
        }
      }      

      if (Math.abs(phiValue - solutionTrspCosts) < 10E-5) { return }

      var cut = cplex.linearNumExpr(solutionTrspCosts)
      
      for( j <- locationIndexes if !openLocs.contains(j)) {
        cut.addTerm(getTransptCosts(openLocs + (j)) - solutionTrspCosts, open(j))
      }

      nbCuts = nbCuts + 1
      add(cplex.ge(cplex.diff(phi, cut), 0))
    }
  }
  
  def solve(): Option[LocationSolution] = {
    var ret: Option[LocationSolution] = None

    val cplex = new IloCplex()

    try {
      val open = Array.tabulate(candidateDCs.size)( i => cplex.boolVar() )
      val phi = cplex.numVar(0, Double.MaxValue)

      val locationCosts = locationIndexes.map { j => cplex.prod(candidateDCs(j).fixedCosts, open(j)) }.fold(cplex.numExpr())(cplex.sum)

      val objective = cplex.sum(locationCosts, phi)
      
      cplex.addMinimize(objective)
      cplex.setOut(null)
      cplex.use(newLazyCutClass(cplex, open, phi))

      cplex.setParam(IloCplex.DoubleParam.TiLim, instructor.timeLimit)
      val begin = System.currentTimeMillis()
      if (cplex.solve()) {
        val end = System.currentTimeMillis()
        val openValues = locationIndexes.map { j => (j, cplex.getValue(open(j))) }.toMap
        val openIndexes = openValues.filter(p => p._2 > 0.5).keys.toSeq
        val openDCs = openIndexes.map { j => candidateDCs(j) }

        val assignments = demandIndexes.map { i => {
          (demands(i), candidateDCs(openIndexes.minBy { j => distance(i)(j) }) )
        } }.toSeq
     
        if(lowerBound < cplex.getBestObjValue)
          lowerBound = cplex.getBestObjValue
          
        var finalGap = (upperBound - lowerBound) / lowerBound
        if(finalGap < 0) finalGap = 0.0
          
        ret = Some(LocationSolution(instance = instance, openDCs = openDCs, assignments = assignments,
          time = 1.0 * (end - begin) / 1000, solver = this, objValue = Math.round(cplex.getObjValue()), gap = finalGap))
      }

    } catch {
      case e: CpxException => println("Cplex exception caught: " + e);
      case NonFatal(e)     => println("exception caught: " + e);
    } finally {
      cplex.end()
    }
    ret
  }
}