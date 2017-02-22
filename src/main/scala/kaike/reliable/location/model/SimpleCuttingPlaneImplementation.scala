package kaike.reliable.location.model

import kaike.reliable.location.data.ReliableProblemInstance
import ilog.cplex.IloCplex
import ilog.concert.IloIntVar
import ilog.concert.IloNumVar
import kaike.reliable.location.data.LocationSolution
import ilog.cplex.CpxException
import ilog.cplex.IloCplex.LazyConstraintCallback
import kaike.reliable.location.data.SolverInstructor
import scala.util.control.NonFatal
import scala.util.control.Breaks._

abstract class SimpleCuttingPlaneImplementation(override val instance: ReliableProblemInstance,
                                                        override val instructor: SolverInstructor,
                                                        override val SOLVER_NAME:String) extends SolverCommon(instance, instructor, SOLVER_NAME){

  val failrate = instance.failRate
  println("Failure probabilities: ")
  println(failrate.map { x => x.toString.padTo(25, " ").mkString }.mkString("[", ", ", "]"))
  
  var nbCuts = 0
  
  var upperBound = Double.MaxValue
  var lowerBound = 0.0
  
  def getTransporationCost(openLocs: IndexedSeq[Int] ): Double
  
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

      beginTime = System.currentTimeMillis()
      
      breakable { while (true) {
        
        if(timeLeft() < 0)
          break;

        cplex.setParam(IloCplex.DoubleParam.TiLim, timeLeft())

        recordNow()
        if (cplex.solve()) {
          val openValues = locationIndexes.map { j => cplex.getValue(open(j)) > 0.5 }
          val openLocs = locationIndexes.filter { j => openValues(j) }
          val setupCosts = openLocs.map { j => candidateDCs(j).fixedCosts }.sum
          val phiValue = cplex.getValue(phi)
          
          println(s"Cutting plane master problem -- obj:${cplex.getObjValue} eta:${phiValue} in ${timeCheckout()}s")
          
          println(s"Current solution ${openLocs.mkString("[", ", ", "]")}")
          
          val trspCosts = getTransporationCost(openLocs)  


          val clb = cplex.getBestObjValue
          val cub = setupCosts + trspCosts

          if (lowerBound < clb) {
            println(s"Lower bound updated: ${lowerBound} -> ${clb}")
            lowerBound = clb
          }

          if (upperBound > cub) {
            println(s"Upper bound updated: ${upperBound} -> ${cub}")
            openDCs = openLocs
            upperBound = cub
          }

          if (lowerBound > 0) {
            if (((upperBound - lowerBound) / lowerBound) < instructor.gap) {
              println("Algorithm terminated due to the gap limit is reached")
              break
            }
          }

          if (phiValue - trspCosts > -EPS) {
            println("Algorithm terminated due to the current eta value is greater than or equals to acctual transportation costs")
            break
          }
          
          if (timeLimitReached()) {
            println("Algorithm terminated due to time limit is reached")
            break
          }          

          var cut = cplex.linearNumExpr(trspCosts)
          var logTerms = Array.empty[String]
          
          for (j <- locationIndexes if !openValues(j)) {
            val newOpenLocs = openLocs :+ j
            val incrementalCost = getTransporationCost(newOpenLocs) - trspCosts
            cut.addTerm(incrementalCost, open(j))
            logTerms = logTerms :+ s"${incrementalCost} * x[${j}] "
          }
          cplex.addGe(phi, cut)
          nbCuts += 1
          println(s"eta >= ${trspCosts} ${logTerms.mkString(" ")} ")
          println(s"Number of cuts added: ${nbCuts} ---------------------------------Time ellasped ${timeUsed}s")

        } else {
          println("Should not come here Error 003")
          println(s"Cutting Plane Master Problem has status ${cplex.getStatus}")
          break
        }

      }}
      

      var finalGap = (upperBound - lowerBound) / lowerBound
      if(finalGap < 0) finalGap = 0.0
    
      
      val assignments = demandIndexes.map { i => {
          (demands(i), candidateDCs(openDCs.minBy { j => distance(i)(j) }) )
        } }.toSeq        
      val status = if( timeLimitReached()) "Time Reached" else  "Gap Reached"
        
      println(s"Final Upper bound: ${upperBound} -- Lower bound: ${lowerBound} --- gap:${finalGap} ----status:${status} ----Time used:${timeUsed}")
      ret = Some(LocationSolution(instance = instance, openDCs = openDCs.map { j => candidateDCs(j) }, assignments = assignments,
        time = timeUsed, solver = this, objValue = Math.round(upperBound), finalGap, status = status))

      
    } catch {
      case e: CpxException => println("Cplex exception caught: " + e);
      case NonFatal(e)     => println("exception caught: " + e);
    } finally {
      cplex.end()
    }
    ret
  }
}