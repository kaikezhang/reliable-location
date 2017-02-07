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

class RUCFLSolver(override val instance: StochasticReliableLocationProblemInstance, override val instructor: SolverInstructor) 
            extends CuttingPlaneLazyConstraintImplementation(instance, instructor, "CuttingPlane for Stocastic RUCFL") {

  def newLazyCutClass(cplex: IloCplex, open: IndexedSeq[IloIntVar], phi: IloNumVar): LazyConstraintCallback = {
    new RUCFLSuperModularCutLazyConstraint(cplex, open, phi)
  }
  
  class RUCFLSuperModularCutLazyConstraint(cplex: IloCplex, open: IndexedSeq[IloIntVar], phi: IloNumVar) extends LazyConstraintCallback {
    def main(): Unit = {
      val openValues = open.map { x => getValue(x) }
      val setupCosts = locationIndexes.map { j => openValues(j) * candidateDCs(j).fixedCosts }.sum
      
      val openDCIndexes = locationIndexes.filter(j => openValues(j) > 0.5)   
      val orderedOpenDCs = demandIndexes.map { i =>
        {
          TreeSet(openDCIndexes:_*)(Ordering.by(j => distance(i)(j)))
        }
      }

      def computeTransporationCosts(dcMatrix: IndexedSeq[TreeSet[Int]]): Double = {
        demandIndexes.map { i =>
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

      val clb = getBestObjValue()
      val cub = setupCosts + solutionTrspCosts
      
      val phiValue = getValue(phi)
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
      
      var cut = cplex.sum(solutionTrspCosts, cplex.numExpr())

      locationIndexes.diff(openDCIndexes).foreach { j =>
        {
          val orderedDCsAddDC = orderedOpenDCs.map { orderDCs => {
            orderDCs + (j) 
          }}
          cut = cplex.sum(cut, cplex.prod(computeTransporationCosts(orderedDCsAddDC) - solutionTrspCosts, open(j) ))
        }
      }
      
      nbCuts = nbCuts + 1
      add(cplex.ge(cplex.diff(phi, cut), 0))
    }
  }

}