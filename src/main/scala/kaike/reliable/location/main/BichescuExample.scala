package kaike.reliable.location.main

import kaike.reliable.location.data.DemandPoint
import kaike.reliable.location.data.CandidateLocation
import kaike.reliable.location.data.ScenariosReliableLocationProblemInstance
import kaike.reliable.location.data.Scenario
import kaike.reliable.location.model.ScenriosBasedRFLPSolver
import kaike.reliable.location.data.SolverInstructor
import kaike.reliable.location.output.NetworkOutput
import kaike.reliable.location.model.RUFLPSolver
import kaike.reliable.location.data.StochasticReliableLocationProblemInstance
import kaike.reliable.location.data.ScenariosStochasticReliableLocationProblemInstance
import kaike.reliable.location.data.ScenariosRobustReliableLocationProblemInstance
import kaike.reliable.location.model.RobustUFLPSolver
import kaike.reliable.location.data.CrossMomentProblemInstance
import kaike.reliable.location.data.ScenariosCrossmomentLocationProblemInstance
import kaike.reliable.location.model.CrossMomentSolver

object BichescuExample {
  
  def casePositiveCorr() = {
    val demand1 = new DemandPoint(0, 100, 10, 2, 4)
    val demand2 = new DemandPoint(1, 100, 10, 4, 4)
    val demand3 = new DemandPoint(2, 100, 10, 6, 4)
    val demand4 = new DemandPoint(3, 100, 10, 1, 6)
    val demand5 = new DemandPoint(4, 100, 10, 1, 2)
    val demand6 = new DemandPoint(5, 100, 10, 7, 6)
    val demand7 = new DemandPoint(6, 100, 10, 7, 2)
    val demands = IndexedSeq(demand1, demand2, demand3,
      demand4, demand5, demand6, demand7)
  
    val location1 = new CandidateLocation(0, 1000, 2, 4)
    val location2 = new CandidateLocation(1, 1000, 4, 4)
    val location3 = new CandidateLocation(2, 1000, 6, 4)
    val candidateLocs = IndexedSeq(location1, location2, location3)
  
    val scenario000 = new Scenario(Seq.empty, 0.8)
    val scenario100 = new Scenario(Seq(0), 0.0)
    val scenario010 = new Scenario(Seq(1), 0)
    val scenario001 = new Scenario(Seq(2), 0.0)
    val scenario110 = new Scenario(Seq(0, 1), 0)
    val scenario101 = new Scenario(Seq(0, 2), 0)
    val scenario011 = new Scenario(Seq(1, 2), 0)
    val scenario111 = new Scenario(Seq(0, 1, 2), 0.2)  
    val scenarios = Seq(scenario000, scenario100, scenario010, scenario001, scenario110, scenario101, scenario011, scenario111)
    
    (demands, candidateLocs, scenarios)
  }
 
  def caseNegativeCorr() = {
    val demand1 = new DemandPoint(0, 100, 10, 2, 4)
    val demand2 = new DemandPoint(1, 100, 10, 4, 4)
    val demand3 = new DemandPoint(2, 100, 10, 6, 4)
    val demand4 = new DemandPoint(3, 100, 10, 1, 6)
    val demand5 = new DemandPoint(4, 100, 10, 1, 2)
    val demand6 = new DemandPoint(5, 100, 10, 7, 6)
    val demand7 = new DemandPoint(6, 100, 10, 7, 2)
    val demands = IndexedSeq(demand1, demand2, demand3,
      demand4, demand5, demand6, demand7)
  
    val location1 = new CandidateLocation(0, 1000, 2, 4)
    val location2 = new CandidateLocation(1, 1000, 4, 4)
    val location3 = new CandidateLocation(2, 1000,  6, 4)
    val candidateLocs = IndexedSeq(location1, location2, location3)
  
    val scenario000 = new Scenario(Seq.empty, 0.2)
    val scenario100 = new Scenario(Seq(0), 0.2)
    val scenario010 = new Scenario(Seq(1), 0.2)
    val scenario001 = new Scenario(Seq(2), 0.2)
    val scenario110 = new Scenario(Seq(0, 1), 0)
    val scenario101 = new Scenario(Seq(0, 2), 0)
    val scenario011 = new Scenario(Seq(1, 2), 0)
    val scenario111 = new Scenario(Seq(0, 1, 2), 0.0)  
    val scenarios = Seq(scenario000, scenario100, scenario010, scenario001, scenario110, scenario101, scenario011, scenario111)
    
    (demands, candidateLocs, scenarios)
  }  

  def runCase(demands: IndexedSeq[DemandPoint], candidateLocs: IndexedSeq[CandidateLocation], scenarios: Seq[Scenario] ) = {
    
    val scenarioInstance = ScenariosReliableLocationProblemInstance(demands, candidateLocs, scenarios)
    val instructor = SolverInstructor()
  
    val scenarioModel = new ScenriosBasedRFLPSolver(scenarioInstance, instructor)  
  
    def runRobustExample() = {
      val instance = new ScenariosRobustReliableLocationProblemInstance(demands, candidateLocs, scenarios)
      val instructor =  SolverInstructor(gap = 0.0) 
      val model = new RobustUFLPSolver(instance, instructor)
      val sol = model.solve().get
  
      showStaticsForSolution(sol.openDCs.map { x => x.index }.toSet)
    }
    def runStochasticExample() = {
      val instance = new ScenariosStochasticReliableLocationProblemInstance(demands, candidateLocs, scenarios)
      val instructor =  SolverInstructor(gap = 0.0) 
      val model = new RUFLPSolver(instance, instructor)
      val sol = model.solve().get
  
      showStaticsForSolution(sol.openDCs.map { x => x.index }.toSet)
    }
    
    def runScenariosExample() = {
      val scenarioInstance = ScenariosReliableLocationProblemInstance(demands, candidateLocs, scenarios)
      val instructor = SolverInstructor()
  
      val model = new ScenriosBasedRFLPSolver(scenarioInstance, instructor)
  
      val sol = model.solve().get
  
      showStaticsForSolution(sol.openDCs.map { x => x.index }.toSet)
    }
    
    def runCrossmomentExample() = {
      val instance = new ScenariosCrossmomentLocationProblemInstance(demands, candidateLocs, scenarios)
      val instructor =  SolverInstructor(gap = 0.0) 
      val model = new CrossMomentSolver(instance, instructor)
      val sol = model.solve().get
  
      showStaticsForSolution(sol.openDCs.map { x => x.index }.toSet)   
    }
    
    def showStaticsForSolution(locs: Set[Int]) = {
      val actualLocationCosts = locs.toSeq.map { i => candidateLocs(i).fixedCosts }.sum
      val actualTranspCosts = scenarioModel.getTransptCostsForScenarios(locs, scenarios)
      
      print(s"Open locations ${locs}".padTo(30, " ").mkString)
      print(s"Location costs: ${actualLocationCosts}".padTo(30, " ").mkString)
      print(s"Transportation costs: ${actualTranspCosts}".padTo(40, " ").mkString)
      print(s"Total Costs: ${actualLocationCosts + actualTranspCosts}".padTo(30, " ").mkString) 
      println()
    }
    
    def enumrateAllsolutions() = {
      val sol100 = Set(0)
      val sol010 = Set(1)
      val sol001 = Set(2)
      val sol110 = Set(0, 1)
      val sol101 = Set(0, 2)
      val sol011 = Set(1, 2)
      val sol111 = Set(0, 1, 2)
      
      val solutions = Seq(sol100, sol010, sol001, sol110, sol101, sol011, sol111)
      
      solutions.foreach(x => showStaticsForSolution(x))
    }
 
    enumrateAllsolutions()
    runScenariosExample()
    runStochasticExample()
    runRobustExample()
    runCrossmomentExample()    
  }

  def main(args: Array[String]): Unit = {
//     runCase _ tupled casePositiveCorr()
     runCase _ tupled caseNegativeCorr()
  }

  
  
}