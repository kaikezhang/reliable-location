package kaike.reliable.location.model

import kaike.reliable.location.data.SolverInstructor
import kaike.reliable.location.data.StochasticReliableLocationProblemInstance
import scala.collection.immutable.TreeSet

class RUFLPSimpleCutSolver(override val instance: StochasticReliableLocationProblemInstance, override val instructor: SolverInstructor) 
            extends SimpleCuttingPlaneImplementation(instance, instructor, "CuttingPlane (simple cut) for Stocastic RUFLP") {
  def getTransporationCost(openLocs: IndexedSeq[Int] ): Double = {
    val orderedOpenDCs = demandIndexes.map { i =>
      {
        TreeSet(openLocs:_*)(Ordering.by(j => distance(i)(j)))
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
    
    
    computeTransporationCosts(orderedOpenDCs)
  }
  
}