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

class RUCFLSolver(val instance: ReliableLocationProblemInstance, val instructor: SolverInstructor) extends Solver("CuttingPlane for Stocastic RUCFL") {
  val demands = instance.demandPoints
  val candidateDCs = instance.candidateLocations
  
  val distance = instance.distance
  val failrate = instance.failRate
  
  val demandIndexes = instance.demandsPointIndexes
  val locationIndexes = instance.candidateLocationIndexes
  
  var nbCuts = 0
  
  var upperBound = Double.MaxValue
  var lowerBound = 0.0    

  class SuperModularCutLazyConstraint(cplex: IloCplex, open: IndexedSeq[IloIntVar], phi: IloNumVar) extends LazyConstraintCallback {
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
     
        if(lowerBound < cplex.getBestObjValue)
          lowerBound = cplex.getBestObjValue
          
        ret = Some(LocationSolution(instance = instance, openDCs = openDCs, assignments = assignments,
          time = 1.0 * (end - begin) / 1000, solver = this, objValue = Math.round(cplex.getObjValue()), gap = (upperBound - lowerBound) / lowerBound ))
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