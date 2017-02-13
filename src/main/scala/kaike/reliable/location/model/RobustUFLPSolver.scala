package kaike.reliable.location.model

import kaike.reliable.location.data.StochasticReliableLocationProblemInstance
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
import kaike.reliable.location.data.RobustReliableLocationProblemInstance

class RobustUFLPSolver(override val instance: RobustReliableLocationProblemInstance, override val instructor: SolverInstructor)
                extends CuttingPlaneLazyConstraintImplementation(instance, instructor, "CuttingPlane for Robust RUFLP") {
  
  def constructWorstCaseScenarios(failureRate: IndexedSeq[Double]): Seq[Scenario] = {
    // sort locations in ascending order of failure rate
    val sortedLocs = locationIndexes.sortBy { j => failureRate(j) }
    // From all failure to non failure, assign proper probability
    val scenarioPattern = locationIndexes.map { j => true }.toArray
    
    val scenarios = collection.mutable.ArrayBuffer.empty[Scenario]
    
    (0 to sortedLocs.size).foreach { j => 
      if(j == 0){
        val failures = locationIndexes.filter { j => scenarioPattern(j) }
        scenarios += new Scenario(failures, failrate(sortedLocs(j)))
      } else if( j == sortedLocs.size){
        scenarioPattern(sortedLocs(j-1)) = false
        val failures = locationIndexes.filter { j => scenarioPattern(j) }
        scenarios += new Scenario(failures, 1.0 -  failrate(sortedLocs(j-1)))          
      } else {
        scenarioPattern(sortedLocs(j-1)) = false
        val failures = locationIndexes.filter { j => scenarioPattern(j) }
        scenarios += new Scenario(failures, failrate(sortedLocs(j)) -  failrate(sortedLocs(j-1)))           
      }
    }
    
    scenarios.toSeq
    
  }
  
  val worstCaseScenarios:Seq[Scenario] = constructWorstCaseScenarios(failrate)
  
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

  class RobustSuperModularCutLazyConstraintt(cplex: IloCplex, open: IndexedSeq[IloIntVar], phi: IloNumVar) 
                                      extends SuperModularCutLazyConstraint(cplex, open, phi){
    override def getTransptCosts(openLocs:Set[Int]):Double = {
      getTransptCostsForScenarios(openLocs, worstCaseScenarios)
    }                                      
  }
  def newLazyCutClass(cplex: IloCplex, open: IndexedSeq[IloIntVar], phi: IloNumVar): LazyConstraintCallback = {
    new RobustSuperModularCutLazyConstraintt(cplex, open, phi)
  }
   
}