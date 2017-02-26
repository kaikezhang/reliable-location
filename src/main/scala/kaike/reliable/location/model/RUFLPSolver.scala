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

class RUFLPSolver(override val instance: StochasticReliableLocationProblemInstance, override val instructor: SolverInstructor) 
            extends CuttingPlaneLazyConstraintImplementation(instance, instructor, "CuttingPlane for Stocastic RUFLP") {

  def newLazyCutClass(cplex: IloCplex, open: IndexedSeq[IloIntVar], phi: IloNumVar): LazyConstraintCallback = {
    new RUCFLSuperModularCutLazyConstraint(cplex, open, phi)
  }
  
  class RUCFLSuperModularCutLazyConstraint(cplex: IloCplex, open: IndexedSeq[IloIntVar], phi: IloNumVar) extends LazyConstraintCallback {
    def main(): Unit = {
      val openValues = open.map { x => getValue(x) }
      val setupCosts = locationIndexes.map { j => openValues(j) * candidateDCs(j).fixedCosts }.sum
      
      val openDCIndexes = locationIndexes.filter(j => openValues(j) > 0.5)
      print(s"Current solution: ${openDCIndexes.mkString("[", ", ", "]")}" )
      
      val phiValue = getValue(phi)
      println(s" with eta value ${phiValue}")      
      
      val orderedOpenDCs = demandIndexes.map { i =>
        {
          TreeSet(openDCIndexes:_*)(Ordering.by(j => distance(i)(j)))
        }
      }

      def computeTransporationCosts(dcMatrix: IndexedSeq[TreeSet[Int]]): Double = {
        demandIndexes.par.map { i =>
          {
            var rate = 1.0
            var costs = 0.0

            dcMatrix(i).foreach { j =>
              {
                costs += rate * (1 - failrate(j)) * demands(i).demand * distance(i)(j)
                rate = rate * failrate(j)
              }
            }
            costs += rate * demands(i).emergencyCost * demands(i).demand
            costs
          }
        }.sum
      }
      
      val solutionTrspCosts = computeTransporationCosts(orderedOpenDCs)
      println(s"Actual transportation costs: ${solutionTrspCosts}")

      val clb = getBestObjValue()
      val cub = setupCosts + solutionTrspCosts
      

      
      if (lowerBound < clb){
        println(s"Lower bound updated ${lowerBound} -> ${clb}")
        lowerBound = clb
      }

      if (upperBound > cub){
        println(s"Upper bound updated ${upperBound} -> ${cub}")
        upperBound = cub
      }

      if (lowerBound > 0) {
        if (((upperBound - lowerBound) / lowerBound) < instructor.gap) {
          println("No cut is added due to gap limit reached.")
          abort()
          return 
        }
      }      

      if (Math.abs(phiValue - solutionTrspCosts) < 10E-5) {
        println("Current eta value is greater than or equals to actual transporation costs. No cut is added.")
        return 
      }
      
      var cut = cplex.sum(solutionTrspCosts, cplex.numExpr())
      
      var logTerms = Array.empty[String]

      locationIndexes.diff(openDCIndexes).foreach { j =>
        {
          val orderedDCsAddDC = orderedOpenDCs.map { orderDCs => {
            orderDCs + (j) 
          }}
          cut = cplex.sum(cut, cplex.prod(computeTransporationCosts(orderedDCsAddDC) - solutionTrspCosts, open(j) ))
          logTerms = logTerms :+ s"${computeTransporationCosts(orderedDCsAddDC) - solutionTrspCosts} * x[${j}] "
        }
      }
      
      nbCuts = nbCuts + 1
//      println(s"eta >= ${solutionTrspCosts} ${logTerms.mkString(" ")} ")
      add(cplex.ge(cplex.diff(phi, cut), 0))
    }
    
    
  }

}