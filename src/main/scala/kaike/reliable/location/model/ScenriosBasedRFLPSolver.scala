package kaike.reliable.location.model

import kaike.reliable.location.data.SolverInstructor
import ilog.cplex.IloCplex
import kaike.reliable.location.data.Scenario
import ilog.concert.IloIntVar
import ilog.concert.IloNumVar
import ilog.cplex.IloCplex.LazyConstraintCallback
import kaike.reliable.location.data.ScenariosReliableLocationProblemInstance

class ScenriosBasedRFLPSolver (override val instance: ScenariosReliableLocationProblemInstance, override val instructor: SolverInstructor) 
            extends CuttingPlaneLazyConstraintImplementation(instance, instructor, "CuttingPlane for Stocastic RUFLP") {

  val scenarios:Seq[Scenario] = instance.scenarios
  
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

  class ScenarioSuperModularCutLazyConstraintt(cplex: IloCplex, open: IndexedSeq[IloIntVar], phi: IloNumVar) 
                                      extends SuperModularCutLazyConstraint(cplex, open, phi){
    override def getTransptCosts(openLocs:Set[Int]):Double = {
      getTransptCostsForScenarios(openLocs, scenarios)
    }                                      
  }
  def newLazyCutClass(cplex: IloCplex, open: IndexedSeq[IloIntVar], phi: IloNumVar): LazyConstraintCallback = {
    new ScenarioSuperModularCutLazyConstraintt(cplex, open, phi)
  }
  
}