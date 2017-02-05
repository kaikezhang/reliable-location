package kaike.reliable.location.model

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
import kaike.reliable.location.data.CrossMonmentProblemInstance
import scala.util.control.Breaks._
import ilog.concert.IloRange
import kaike.reliable.location.data.Scenario

class CrossMonmentSolver(val instance: CrossMonmentProblemInstance, val instructor: SolverInstructor) extends Solver("CrossMonment") {
  val demands = instance.demandPoints
  val candidateDCs = instance.candidateLocations

  val distance = instance.distance
  val failrate = instance.failRate
  val crossMonmentMatrix = instance.crossMonmentMatrix

  val demandIndexes = instance.demandsPointIndexes
  val locationIndexes = instance.candidateLocationIndexes

  val randomRealizations = instance.realizations

  val gapLimit = instructor.gap

  var nbCuts = 0

  var upperBound = Double.MaxValue
  var lowerBound = 0.0

  var InitialFeasibleScenarios = Array.empty[Scenario]

  var overallBeginTime: Double = _
  var openDCs = Seq.empty[Int]

  def timeUsed(): Double = {
    1.0 * (System.currentTimeMillis() - overallBeginTime) / 1000
  }
  def timeLeft(): Double = {
    instructor.timeLimit - timeUsed
  }

  def timeLimitReached(): Boolean = {
    timeLeft < 0
  }

  def solve(): Option[LocationSolution] = {
    var ret: Option[LocationSolution] = None

    val cuttingPlaneMainProblem = new IloCplex()
    cuttingPlaneMainProblem.setOut(null)

    try {
      val open = locationIndexes.map(j => cuttingPlaneMainProblem.boolVar())
      val phi = cuttingPlaneMainProblem.numVar(0, Double.MaxValue)

      val locationCosts = cuttingPlaneMainProblem.linearNumExpr()
      locationIndexes.foreach { j =>
        locationCosts.addTerm(candidateDCs(j).fixedCosts, open(j))
      }

      val objective = cuttingPlaneMainProblem.sum(locationCosts, phi)
      cuttingPlaneMainProblem.addMinimize(objective)

      overallBeginTime = System.currentTimeMillis()
      breakable { while (true) {
        if (timeLimitReached()) {
          break
        }
        cuttingPlaneMainProblem.setParam(IloCplex.DoubleParam.TiLim, timeLeft())

        cuttingPlaneMainProblem.solve()

        if (cuttingPlaneMainProblem.getStatus == IloCplex.Status.Optimal) {
          val openValues = locationIndexes.map { j => cuttingPlaneMainProblem.getValue(open(j)) > 0.5 }
          val openLocs = locationIndexes.filter { j => openValues(j) }
          val setupCosts = openLocs.map { j => candidateDCs(j).fixedCosts }.sum
          val phiValue = cuttingPlaneMainProblem.getValue(phi)
          val (trspCosts, scenarios) = SEPERATE(openValues)

          if (scenarios.size == 0) {
            println("Time limit reached during solving seperation problem.")
            break
          }

          val clb = cuttingPlaneMainProblem.getBestObjValue
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
            if (((upperBound - lowerBound) / lowerBound) < gapLimit) {
              println("No cut is added due to gap limit reached.")
              break
            }
          }

          if (phiValue - trspCosts > -1E-6) {
            println("No cut is added due to current phi is greater than or equals to acctual transportation costs")
            break
          }

          var cut = cuttingPlaneMainProblem.linearNumExpr(trspCosts)

          for (j <- locationIndexes if !openValues(j)) {
            val newOpenLocs = openLocs :+ j
            val incrementalCost = getTransptCostsForScenarios(newOpenLocs.toSet, scenarios) - trspCosts
            cut.addTerm(incrementalCost, open(j))
          }

          cuttingPlaneMainProblem.addGe(phi, cut)
          nbCuts += 1

        } else {
          println("Should not come here Erro 003")
          println(s"Cplex status ${cuttingPlaneMainProblem.getStatus}")
          break
        }

      }}
      
      println(s"Upper bound: ${upperBound} -- Lower bound: ${lowerBound}")
      // conclude solution status
      
      val assignments = demandIndexes.map { i => {
          (demands(i), candidateDCs(openDCs.minBy { j => distance(i)(j) }) )
        } }.toSeq        
      
      ret = Some(LocationSolution(instance = instance, openDCs = openDCs.map { j => candidateDCs(j) }, assignments = assignments,
        time = timeUsed, solver = this, objValue = Math.round(upperBound), (upperBound - lowerBound)/ lowerBound))      

    } catch {
      case e: CpxException => {
        println("Cplex exception caught: " + e);
        e.printStackTrace()
      }
      case NonFatal(e)     => println("exception caught: " + e);
    } finally {
      cuttingPlaneMainProblem.end()
    }
    ret
  }

  def SEPERATE(openValues: IndexedSeq[Boolean]): (Double, Seq[Scenario]) = {

    var terminatedDueToTimeLimit = false

    var sepDual_objValue = Double.MaxValue

    val openLocs = locationIndexes.filter { j => openValues(j) }.toSet

    println(s"Seperating solution ${openLocs}")

    val scenarios = collection.mutable.ArrayBuffer.empty[Scenario]

    val modelZsepDual = new IloCplex()

    val alpha = modelZsepDual.numVar(-Double.MaxValue, Double.MaxValue)
    val beta = locationIndexes.map { j => modelZsepDual.numVar(-Double.MaxValue, Double.MaxValue) }
    val betabar = (for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMonmentMatrix(j1)(j2) > 0)
      yield ((j1, j2), modelZsepDual.numVar(-Double.MaxValue, Double.MaxValue))).toMap
    
    val objCosts = modelZsepDual.linearNumExpr()
    objCosts.addTerm(1.0, alpha)

    locationIndexes.foreach { j => objCosts.addTerm(crossMonmentMatrix(j)(j), beta(j)) }

    for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMonmentMatrix(j1)(j2) > 0 ) {
      objCosts.addTerm(crossMonmentMatrix(j1)(j2), betabar(j1, j2))
    }

    modelZsepDual.setParam(IloCplex.IntParam.RootAlg, IloCplex.Algorithm.Primal)
    modelZsepDual.setOut(null)    
    modelZsepDual.setParam(IloCplex.BooleanParam.PreInd, false)
    modelZsepDual.addMinimize(objCosts)

    val Cuts = collection.mutable.ArrayBuffer.empty[IloRange]

    def modelZsepDualAddCutForScenario(scenario: Scenario) = {
      val cutlhs = modelZsepDual.linearNumExpr()
      cutlhs.addTerm(1.0, alpha)
      scenario.failures.foreach { k => cutlhs.addTerm(1.0, beta(k)) }
      
      for (j1 <- scenario.failures; j2 <- scenario.failures if j1 < j2 && crossMonmentMatrix(j1)(j2) > 0) {
        cutlhs.addTerm(1.0, betabar(j1, j2))
      }

      val cut = modelZsepDual.addGe(cutlhs, getTransptCostsForScenario(openLocs, scenario))
      Cuts += cut
      scenarios += scenario
    }

    if(InitialFeasibleScenarios.size > 0 ){
      InitialFeasibleScenarios.foreach { scenario =>
        modelZsepDualAddCutForScenario(scenario)
      }
    } else {
      randomRealizations.foreach { scenario =>
        modelZsepDualAddCutForScenario(scenario)
      }      
    }
      

    // build pricing model

    val pricingModel = new IloCplex()
    pricingModel.setOut(null)
    
    val pi = demandIndexes.map { i => pricingModel.numVar(-Double.MaxValue, Double.MaxValue) }
    val xi = locationIndexes.map { i => pricingModel.boolVar() }
    val lambda = (for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMonmentMatrix(j1)(j2) > 0)
      yield ((j1, j2), pricingModel.numVar(0, 1))).toMap

    for (i <- demandIndexes; j <- locationIndexes) {
      var lhs = pricingModel.numExpr()
      val pi_eff = if(openLocs.contains(j)) -distance(i)(j) * demands(i).demand else -demands(i).emergencyCost * demands(i).demand
      lhs = pricingModel.sum(lhs, pi(i))
      lhs = pricingModel.sum(lhs, pricingModel.prod(pi_eff, pricingModel.diff(1, xi(j))))
      lhs = pricingModel.sum(lhs, pricingModel.prod(-demands(i).emergencyCost * demands(i).demand, xi(j)))
      pricingModel.addLe(lhs, 0)
    }

    for (i <- demandIndexes) {
      pricingModel.addLe(pi(i), demands(i).emergencyCost * demands(i).demand)
    }

    for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMonmentMatrix(j1)(j2) > 0) {
      pricingModel.addLe(lambda(j1, j2), xi(j1))
      pricingModel.addLe(lambda(j1, j2), xi(j2))
      pricingModel.addGe(lambda(j1, j2), pricingModel.sum(-1.0, pricingModel.sum(xi(j1), xi(j2))))
    }

    val ReducedCostExpr = pricingModel.linearNumExpr()
    val ReducedCost = pricingModel.addMaximize(ReducedCostExpr)

    demandIndexes.foreach { i => ReducedCostExpr.addTerm(1.0, pi(i)) }

    if (InitialFeasibleScenarios.size == 0) {
      val unboundedModel = new IloCplex()
      val xi = locationIndexes.map { j => unboundedModel.boolVar() }
      val lambda = (for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMonmentMatrix(j1)(j2) > 0)
      yield ((j1, j2), unboundedModel.numVar(0, 1))).toMap

      for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMonmentMatrix(j1)(j2) > 0) {
        unboundedModel.addLe(lambda(j1, j2), xi(j1))
        unboundedModel.addLe(lambda(j1, j2), xi(j2))
        unboundedModel.addGe(lambda(j1, j2), unboundedModel.sum(-1.0, unboundedModel.sum(xi(j1), xi(j2))))
      }
      val unboundedReducedCostLinearExpr = unboundedModel.linearNumExpr()
      val unboundedReducedCost = unboundedModel.addMinimize(unboundedReducedCostLinearExpr)
      unboundedModel.setOut(null)
      unboundedModel.setWarning(null)

      def generateInfeasibilityCut(): Scenario = {
        val extremRay = modelZsepDual.getRay()
        val rayIterator = extremRay.linearIterator()
        val rayMap = collection.mutable.Map.empty[IloNumVar, Double]
        while (rayIterator.hasNext()) {
          val variable = rayIterator.nextNumVar()
          rayMap += ((variable, rayIterator.getValue()))
        }
        unboundedReducedCostLinearExpr.clear()
        unboundedReducedCostLinearExpr.setConstant(rayMap.getOrElse(alpha, 0.0))
        
//        println(s"Ray alpha -- ${rayMap.getOrElse(alpha, 0.0)}")
        for (j <- locationIndexes) {
          unboundedReducedCostLinearExpr.addTerm(rayMap.getOrElse(beta(j), 0.0), xi(j))
        }
        for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMonmentMatrix(j1)(j2) > 0) {
          unboundedReducedCostLinearExpr.addTerm(rayMap.getOrElse(betabar(j1, j2), 0.0), lambda(j1, j2))
        }

        unboundedReducedCost.setExpr(unboundedReducedCostLinearExpr)
        unboundedModel.solve()

        if (unboundedModel.getStatus() == IloCplex.Status.Optimal) {
          if (unboundedModel.getObjValue() < 1E-6) {
//            println(s"${unboundedModel.getObjValue()}")
            val xiValues = xi.map { xi_i => (unboundedModel.getValue(xi_i) + 0.5).toInt }
            val failurePattern = locationIndexes.filter { j => xiValues(j) > 0.5 }

            if (Math.abs(unboundedModel.getObjValue()) < 1E-6) {
              val integerCut = unboundedModel.linearNumExpr()
              for (j <- locationIndexes) {
                xiValues(j) match {
                  case 0 => integerCut.addTerm(1.0, xi(j))
                  case 1 => integerCut.addTerm(-1.0, xi(j))
                }
              }
              integerCut.setConstant(xiValues.sum)
              unboundedModel.addGe(integerCut, 1.0)
            }

            return new Scenario(failurePattern.toSet, 0.0)
          } else {
            println(s"UnboundedModel status: ${unboundedModel.getStatus}")
            println("Should not come here Erro 002x1")
          }
        }

        println("Should not come here Erro 002x")
        new Scenario(Set.empty, 0.0)
      }

      println("Generating initial scenarios if necessary")

      breakable { while (true) {
        modelZsepDual.solve()        
        if (modelZsepDual.getStatus == IloCplex.Status.Optimal) {
          (0 until Cuts.size).foreach(i => {
            scenarios(i).prob = modelZsepDual.getDual(Cuts(i))
          })
          InitialFeasibleScenarios = scenarios.filter { x => x.prob > 0 }.toArray
//          InitialFeasibleScenarios.foreach{ x =>
//            println(s"${x.failures} with prob ${x.prob}")
//          }
          break
        } else if (modelZsepDual.getStatus == IloCplex.Status.Unbounded || 
            modelZsepDual.getStatus == IloCplex.Status.InfeasibleOrUnbounded) {
          val scenario = generateInfeasibilityCut()
          modelZsepDualAddCutForScenario(scenario)
        } else {
          println(modelZsepDual.getStatus)
          println("Should not come here: Erro 001")
        }

        if (timeLimitReached()) {
          terminatedDueToTimeLimit = true
          break
        }
      }}
      println("End of generating initial scenarios")
      unboundedModel.end()
    }

    breakable { while (true) {
      if (timeLimitReached()) {
        terminatedDueToTimeLimit = true
        break
      }

      modelZsepDual.solve()
      
      def solvePricingProblem(): Option[Scenario] = {
        var ret: Option[Scenario] = None
        val alphaValue = modelZsepDual.getValue(alpha)
        val betaValues = beta.map { beta_i => modelZsepDual.getValue(beta_i) }
        val betabarValues = betabar.mapValues { variable => modelZsepDual.getValue(variable) }
        ReducedCostExpr.setConstant(-1.0 * alphaValue)

        for (j <- locationIndexes) {
          ReducedCostExpr.remove(xi(j))
          ReducedCostExpr.addTerm(-1.0 * betaValues(j), xi(j))
        }

        for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMonmentMatrix(j1)(j2) > 0) {
          ReducedCostExpr.remove(lambda(j1, j2))
          ReducedCostExpr.addTerm(-1.0 * betabarValues(j1, j2), lambda(j1, j2))
        }

        ReducedCost.setExpr(ReducedCostExpr)
        pricingModel.solve()

        if (pricingModel.getStatus == IloCplex.Status.Optimal) {
//          println(s"Pricing problem objective value: ${pricingModel.getObjValue}")
          if (pricingModel.getObjValue() > 1E-6) {
            val xiValues = xi.map { xi_i => pricingModel.getValue(xi_i) }
            val failurePattern = locationIndexes.filter { j => xiValues(j) > 0.5 }
            ret = Option(new Scenario(failurePattern.toSet, 0.0))
          } else {
//            println("Pricing problem has non positive objective value.")
          }
        } else {
          println(s"PricingModel status: ${pricingModel.getStatus}")
          println("Should not come here Error 01TX")
        }
        ret
      }

      if(modelZsepDual.getStatus == IloCplex.Status.Optimal){
      solvePricingProblem() match {
        case Some(scenario) => {
          modelZsepDualAddCutForScenario(scenario)
        }
        case _ => break
      }} else {
        break
      }

    }}

    if (modelZsepDual.getStatus == IloCplex.Status.Optimal) {
      sepDual_objValue = modelZsepDual.getObjValue
      (0 until Cuts.size).foreach(i => {
        scenarios(i).prob = modelZsepDual.getDual(Cuts(i))
      })
    } else {
      println("ModelZsepDual status: " + modelZsepDual.getStatus)
      println("Should not come here Erro 003X2")
    }

    pricingModel.end()
    modelZsepDual.end()

    if (terminatedDueToTimeLimit)
      (-1.0, Seq.empty)
    else
      (sepDual_objValue, scenarios.filter { p => p.prob > 0 })
  }

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

}