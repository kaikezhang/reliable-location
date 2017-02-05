package kaike.reliable.location.model

import kaike.reliable.location.data.ReliableLocationProblemInstance
import kaike.reliable.location.data.SolverInstructor
import ilog.cplex.IloCplex
import kaike.reliable.location.data.LocationSolution
import ilog.cplex.CpxException
import scala.util.control.NonFatal
import ilog.cplex.IloCplex.LazyConstraintCallback
import ilog.concert.IloIntVar
import kaike.reliable.location.data.CandidateLocation
import ilog.concert.IloNumVar
import kaike.reliable.location.data.DemandPoint
import scala.collection.immutable.TreeSet
import kaike.reliable.location.data.Scenario
import kaike.reliable.location.data.RobustLocationProblemInstance

class RobustUCFLSolver(val instance: RobustLocationProblemInstance, val instructor: SolverInstructor) extends Solver("Robust-UCFL") {
  val demands = instance.demandPoints
  val candidateDCs = instance.candidateLocations
  
  val distance = instance.distance
  val failrate = instance.failRate
  
  val demandIndexes = instance.demandsPointIndexes
  val locationIndexes = instance.candidateLocationIndexes
  
  var upperBound = Double.MaxValue
  var lowerBound = 0.0  
  
  def constructWorstCaseScenarios(failureRate: IndexedSeq[Double]): Seq[Scenario] = {
    // sort locations in ascending order of failure rate
    val sortedLocs = locationIndexes.sortBy { j => failureRate(j) }
    // From all failure to non failure, assign proper probability
    val scenarioPattern = locationIndexes.map { j => true }.toArray
    
    val scenarios = collection.mutable.ArrayBuffer.empty[Scenario]
    
    (0 to sortedLocs.size).foreach { j => 
      if(j == 0){
        val failures = locationIndexes.filter { j => scenarioPattern(j) }.toSet
        scenarios += new Scenario(failures, failrate(sortedLocs(j)))
      } else if( j == sortedLocs.size){
        scenarioPattern(sortedLocs(j-1)) = false
        val failures = locationIndexes.filter { j => scenarioPattern(j) }.toSet
        scenarios += new Scenario(failures, 1.0 -  failrate(sortedLocs(j-1)))          
      } else {
        scenarioPattern(sortedLocs(j-1)) = false
        val failures = locationIndexes.filter { j => scenarioPattern(j) }.toSet
        scenarios += new Scenario(failures, failrate(sortedLocs(j)) -  failrate(sortedLocs(j-1)))           
      }
    }
    
    scenarios.toSeq
    
  }
  
  val worstCaseScenarios:Seq[Scenario] = constructWorstCaseScenarios(failrate)
  
  var nbCuts = 0
  
  def getTransptCostsForScenario(openLocs: Set[Int], scenario: Scenario): Double = {
    val effectiveLocs = openLocs.filter { j => !scenario.failures.contains(j) }
    if (effectiveLocs.size == 0) {
      demandIndexes.map { i => demands(i).emergencyCost * demands(i).demand }.sum
    } else {
      demandIndexes.map { i => effectiveLocs.map(j => distance(i)(j)).min * demands(i).demand }.sum
    }
  }

  def getTransptCostsForScenarios(openLocs: Set[Int], scenarios: Seq[Scenario]): Double = {
    scenarios.par.map { scenario => scenario.prob * getTransptCostsForScenario(openLocs, scenario) }.sum
  }  

  class SuperModularCutLazyConstraint(cplex: IloCplex, open: IndexedSeq[IloIntVar], phi: IloNumVar) extends LazyConstraintCallback {
    def main(): Unit = {
      val openValues = open.map { x => getValue(x) }
      val setupCosts = locationIndexes.map { j => openValues(j) * candidateDCs(j).fixedCosts }.sum
      
      val openLocs = locationIndexes.filter(j => openValues(j) > 0.5).toSet 

      
      val solutionTrspCosts = getTransptCostsForScenarios(openLocs, worstCaseScenarios )


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
        cut.addTerm(getTransptCostsForScenarios(openLocs + (j), worstCaseScenarios ) - solutionTrspCosts, open(j))
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
      cplex.use(new SuperModularCutLazyConstraint(cplex, open, phi))

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
        
        ret = Some(LocationSolution(instance = instance, openDCs = openDCs, assignments = assignments,
          time = 1.0 * (end - begin) / 1000, solver = this, objValue = Math.round(cplex.getObjValue()) ))
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