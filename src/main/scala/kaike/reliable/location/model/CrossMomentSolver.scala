package kaike.reliable.location.model

import scala.collection.immutable.TreeSet
import scala.util.control.Breaks.break
import scala.util.control.Breaks.breakable
import scala.util.control.NonFatal

import ilog.concert.IloNumVar
import ilog.concert.IloRange
import ilog.cplex.CpxException
import ilog.cplex.IloCplex
import kaike.reliable.location.data.CrossMomentProblemInstance
import kaike.reliable.location.data.LocationSolution
import kaike.reliable.location.data.Scenario
import kaike.reliable.location.data.SolverInstructor

class CrossMomentSolver(override val instance: CrossMomentProblemInstance, override val instructor: SolverInstructor) extends CrossMomentSolverAbstract(instance, instructor, "CuttingPlane + ColumnGen") {

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

      beginTime = System.currentTimeMillis()

      generateInitialScenarios()

      breakable {
        while (!modelInfeasible) {

          if (timeLeft() < 0)
            break;

          cuttingPlaneMainProblem.setParam(IloCplex.DoubleParam.TiLim, timeLeft())

          recordNow()
          if (cuttingPlaneMainProblem.solve()) {
            val openValues = locationIndexes.map { j => cuttingPlaneMainProblem.getValue(open(j)) > 0.5 }
            val openLocs = locationIndexes.filter { j => openValues(j) }
            val setupCosts = openLocs.map { j => candidateDCs(j).fixedCosts }.sum
            val phiValue = cuttingPlaneMainProblem.getValue(phi)

            val clb = cuttingPlaneMainProblem.getBestObjValue

            if (lowerBound < clb) {
              println(s"Lower bound updated: ${lowerBound} -> ${clb}")
              lowerBound = clb
            }

            def addSupermodularCut(transpCosts: Double, scenarios: Seq[Scenario]) = {
              var cut = cuttingPlaneMainProblem.linearNumExpr(transpCosts)

              for (j <- locationIndexes if !openValues(j)) {
                val newOpenLocs = openLocs :+ j
                val incrementalCost = getTransptCostsForScenarios(newOpenLocs.toSet, scenarios) - transpCosts
                cut.addTerm(incrementalCost, open(j))
              }
              cuttingPlaneMainProblem.addGe(phi, cut)
              nbCuts += 1
              println(s"Number of Cuts (for cutting plane master problem) added: ${nbCuts} ---------------------------------Time ellasped ${timeUsed}s")
            }

            println(s"Cutting plane master problem -- obj:${cuttingPlaneMainProblem.getObjValue} phi:${phiValue} in ${timeCheckout()}s")

            println(s"Seperate solution ${openLocs.to[TreeSet]}")

            val optimisticEstTransportationCosts = getTransptCostsForScenarios(openLocs.toSet, InitialScenarios)
            print(s"Attempt to use optimistic cut with estimation of transportation costs ${optimisticEstTransportationCosts}")

            if (optimisticEstTransportationCosts > phiValue + EPS) {
              println("  success: optimistic cut applied")
              addSupermodularCut(optimisticEstTransportationCosts, InitialScenarios)
            } else {

              recordNow("Seperation")
              val (trspCosts, scenarios) = SEPERATE(openValues = openValues)
              println(s"Seperation problem is solved in ${timeCheckout("Seperation")}s --- transportation costs: ${trspCosts} for solution ${openLocs}")

              if (scenarios.size == 0) {
                println("Time limit is reached during solving seperation problem.")
                break
              }

              val cub = setupCosts + trspCosts

              if (upperBound > cub) {
                println(s"Upper bound updated: ${upperBound} -> ${cub}")
                openDCs = openLocs
                upperBound = cub
              }

              if (lowerBound > 0) {
                if (((upperBound - lowerBound) / lowerBound) < instructor.gap) {
                  println("No cut is added due to the gap limit is reached")
                  break
                }
              }

              if (phiValue - trspCosts > -EPS) {
                println("No cut is added due to the current phi value is greater than or equals to acctual transportation costs")
                break
              }

              if (timeLimitReached()) {
                println("Time limit is reached break the loop")
                break
              }
              InitialScenarios = scenarios.toArray
              addSupermodularCut(trspCosts, scenarios)
            }
          } else {
            throw new Exception(s"Cutting Plane Master Problem has status ${cuttingPlaneMainProblem.getStatus} when it should be solved.")
          }

        }
      }

      if (!modelInfeasible) {
        var finalGap = (upperBound - lowerBound) / lowerBound
        if (finalGap < 0) finalGap = 0.0

        val assignments = demandIndexes.map { i =>
          {
            (demands(i), candidateDCs(openDCs.minBy { j => distance(i)(j) }))
          }
        }.toSeq
        val status = if (timeLimitReached()) "Time Reached" else "Gap Reached"

        println(s"Final Upper bound: ${upperBound} -- Lower bound: ${lowerBound} --- gap:${finalGap} ----status:${status} ----Time used:${timeUsed}")
        ret = Some(LocationSolution(instance = instance, openDCs = openDCs.map { j => candidateDCs(j) }, assignments = assignments,
          time = timeUsed, solver = this, objValue = Math.round(upperBound), finalGap, status = status))
      } else {
        nbCuts = 0
        println("Cross moment matrix is infeasible")
        ret = Some(LocationSolution(instance = instance, openDCs = Seq.empty, assignments = Seq.empty,
          time = timeUsed, solver = this, objValue = 0, gap = 0, status = "Infeasible"))
      }

    } catch {
      case e: CpxException => {
        println("Cplex exception caught: " + e);
        e.printStackTrace(Console.out)
      }
      case NonFatal(e) => {
        println("exception caught: " + e);
        e.printStackTrace(Console.out)
      }
    } finally {
      cuttingPlaneMainProblem.end()
    }
    ret
  }

  def generateInitialScenarios() = {
    val openValues: IndexedSeq[Boolean] = locationIndexes.map { x => false }
    SEPERATE(openValues = openValues, generateIntitialScenario = true)
  }

  def SEPERATE(openValues: IndexedSeq[Boolean], generateIntitialScenario: Boolean = false): (Double, Seq[Scenario]) = {

    var terminatedDueToTimeLimit = false

    var sepDual_objValue = Double.MaxValue

    val openLocs = TreeSet(locationIndexes.filter { j => openValues(j) }: _*)

    val patternsInModelZsepDual = collection.mutable.HashSet.empty[Set[Int]]

    val scenarios = collection.mutable.ArrayBuffer.empty[Scenario]

    val modelZsepDual = new IloCplex()

    val alpha = modelZsepDual.numVar(-Double.MaxValue, Double.MaxValue)
    val beta = locationIndexes.map { j => modelZsepDual.numVar(-Double.MaxValue, Double.MaxValue) }
    val betabar = (for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMomentMatrix(j1)(j2) > -EPS)
      yield ((j1, j2), modelZsepDual.numVar(-Double.MaxValue, Double.MaxValue))).toMap

    val objCosts = modelZsepDual.linearNumExpr()
    objCosts.addTerm(SCALE_FACTOR, alpha)

    locationIndexes.foreach { j => objCosts.addTerm(crossMomentMatrix(j)(j), beta(j)) }

    for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMomentMatrix(j1)(j2) > -EPS) {
      objCosts.addTerm(crossMomentMatrix(j1)(j2), betabar(j1, j2))
    }

    //    modelZsepDual.setParam(IloCplex.IntParam.RootAlg, IloCplex.Algorithm.Primal)
    modelZsepDual.setOut(null)
    //Turn off preprocessing otherwise the model will return status of infeasibleorunbounded
    modelZsepDual.setParam(IloCplex.BooleanParam.PreInd, false)
    modelZsepDual.addMinimize(objCosts)

    val Cuts = collection.mutable.ArrayBuffer.empty[IloRange]

    def modelZsepDualAddCutForScenario(scenario: Scenario) = {
      val cutlhs = modelZsepDual.linearNumExpr()
      cutlhs.addTerm(1.0, alpha)
      scenario.failures.foreach { k => cutlhs.addTerm(1.0, beta(k)) }

      for (j1 <- scenario.failures; j2 <- scenario.failures if j1 < j2 && crossMomentMatrix(j1)(j2) > -EPS) {
        cutlhs.addTerm(1.0, betabar(j1, j2))
      }

      val cut = modelZsepDual.addGe(cutlhs, getTransptCostsForScenario(openLocs, scenario))
      Cuts += cut
      scenarios += scenario
      patternsInModelZsepDual += scenario.failures
    }

    if (generateIntitialScenario) {
      ArtificialSeedRealizations.foreach { scenario =>
        modelZsepDualAddCutForScenario(scenario)
      }
    } else {
      InitialScenarios.foreach { scenario =>
        modelZsepDualAddCutForScenario(scenario)
      }
    }

    // build pricing model

    val pricingModel = new IloCplex()
    pricingModel.setOut(null)

    val pi = demandIndexes.map { i => pricingModel.numVar(-Double.MaxValue, Double.MaxValue) }
    val xi = locationIndexes.map { i => pricingModel.boolVar() }
    val lambda = (for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMomentMatrix(j1)(j2) > -EPS)
      yield ((j1, j2), pricingModel.numVar(0, 1))).toMap

    for (i <- demandIndexes; j <- locationIndexes) {
      var lhs = pricingModel.numExpr()
      val pi_eff = if (openLocs.contains(j)) -distance(i)(j) * demands(i).demand else -demands(i).emergencyCost * demands(i).demand
      lhs = pricingModel.sum(lhs, pi(i))
      lhs = pricingModel.sum(lhs, pricingModel.prod(pi_eff, pricingModel.diff(1, xi(j))))
      lhs = pricingModel.sum(lhs, pricingModel.prod(-demands(i).emergencyCost * demands(i).demand, xi(j)))
      pricingModel.addLe(lhs, 0)
    }

    for (i <- demandIndexes) {
      pricingModel.addLe(pi(i), demands(i).emergencyCost * demands(i).demand)
    }

    for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMomentMatrix(j1)(j2) > -EPS) {
      pricingModel.addLe(lambda(j1, j2), xi(j1))
      pricingModel.addLe(lambda(j1, j2), xi(j2))
      pricingModel.addGe(lambda(j1, j2), pricingModel.sum(-1.0, pricingModel.sum(xi(j1), xi(j2))))
    }

    val ReducedCostExpr = pricingModel.linearNumExpr()
    val ReducedCost = pricingModel.addMaximize(ReducedCostExpr)

    demandIndexes.foreach { i => ReducedCostExpr.addTerm(1.0, pi(i)) }

    if (generateIntitialScenario) {
      val unboundedModel = new IloCplex()
      val xi = locationIndexes.map { j => unboundedModel.boolVar() }
      val lambda = (for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMomentMatrix(j1)(j2) > -EPS)
        yield ((j1, j2), unboundedModel.numVar(0, 1))).toMap

      for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMomentMatrix(j1)(j2) > -EPS) {
        unboundedModel.addLe(lambda(j1, j2), xi(j1))
        unboundedModel.addLe(lambda(j1, j2), xi(j2))
        unboundedModel.addGe(lambda(j1, j2), unboundedModel.sum(-1.0, unboundedModel.sum(xi(j1), xi(j2))))
      }
      val unboundedReducedCostLinearExpr = unboundedModel.linearNumExpr()
      val unboundedReducedCost = unboundedModel.addMinimize(unboundedReducedCostLinearExpr)
      unboundedModel.setOut(null)
      unboundedModel.setWarning(null)

      def generateInfeasibilityCut(): Option[Scenario] = {
        var ret: Option[Scenario] = None
        val extremRay = modelZsepDual.getRay()
        val rayIterator = extremRay.linearIterator()
        val rayMap = collection.mutable.Map.empty[IloNumVar, Double]
        while (rayIterator.hasNext()) {
          val variable = rayIterator.nextNumVar()
          rayMap += ((variable, rayIterator.getValue()))
        }
        unboundedReducedCostLinearExpr.clear()
        unboundedReducedCostLinearExpr.setConstant(rayMap.getOrElse(alpha, 0.0))

        for (j <- locationIndexes) {
          unboundedReducedCostLinearExpr.addTerm(rayMap.getOrElse(beta(j), 0.0), xi(j))
        }
        for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMomentMatrix(j1)(j2) > -EPS) {
          unboundedReducedCostLinearExpr.addTerm(rayMap.getOrElse(betabar(j1, j2), 0.0), lambda(j1, j2))
        }

        unboundedReducedCost.setExpr(unboundedReducedCostLinearExpr)

        unboundedModel.solve()

        if (unboundedModel.getStatus() == IloCplex.Status.Optimal) {
          print(s"Pricing problem for unbounded ZsepDual solved in ${timeCheckout()}s with status ${unboundedModel.getStatus()} --- reduced costs = ${unboundedModel.getObjValue()}".padTo(120, " ").mkString)
          if (unboundedModel.getObjValue() < -EPS) {
            val xiValues = xi.map { xi_i => (unboundedModel.getValue(xi_i) + 0.5).toInt }
            val failurePattern = locationIndexes.filter { j => xiValues(j) > 0.5 }
            println(s"Scenario added ${failurePattern}")
            ret = Option(new Scenario(failurePattern, 0.0))
          } else {
            println()
            println(s"Cross moment matrix is infeasible")
          }
        } else {
          throw new Exception(s"Pricing problem for unbound sepDual got status ${unboundedModel.getStatus()} when it should be solved to optimal.")
        }
        ret
      }

      println("Generating feasible scenarios")

      breakable {
        while (true) {
          recordNow()
          modelZsepDual.solve()
          print(s"Model ZsepDual solved in ${timeCheckout()}s with status ${modelZsepDual.getStatus}".padTo(60, " ").mkString)
          if (modelZsepDual.getStatus == IloCplex.Status.Optimal) {
            break
          } else if (modelZsepDual.getStatus == IloCplex.Status.Unbounded ||
            modelZsepDual.getStatus == IloCplex.Status.InfeasibleOrUnbounded) {
            generateInfeasibilityCut() match {
              case Some(scenario) => {
                modelZsepDualAddCutForScenario(scenario)
              }
              case _ => {
                modelInfeasible = true
                break
              }
            }

          } else {
            throw new Exception(s"PricingModel ended up with status ${pricingModel.getStatus} when it should be solved to optimal.")
          }

          if (timeLimitReached()) {
            terminatedDueToTimeLimit = true
            break
          }
        }
      }
      println("End of generating feasible scenarios")
      println()
      unboundedModel.end()
    }

    if (generateIntitialScenario) {
      (0 until Cuts.size).foreach(i => {
        scenarios(i).prob = modelZsepDual.getDual(Cuts(i)) / SCALE_FACTOR
      })
      InitialScenarios = scenarios.filter { x => x.prob > 0 }.toArray

      InitialScenarios.foreach { x => println(s"${x.failures} with probability ${x.prob}") }
      println()
      return (0, InitialScenarios.filter { p => p.prob > 0 })
    }

    var lastAddedPattern = Set.empty[Int]
    var currentObj = 0.0

    pricingModel.setParam(IloCplex.IntParam.SolnPoolCapacity, 10)
    pricingModel.setParam(IloCplex.IntParam.PopulateLim, 100)

    recordNow("pricing")
    breakable {
      while (!modelInfeasible) {

        def solvePricingProblem(): Option[Seq[Scenario]] = {
          var ret: Option[Seq[Scenario]] = None
          val alphaValue = modelZsepDual.getValue(alpha)
          val betaValues = beta.map { beta_i => modelZsepDual.getValue(beta_i) }
          val betabarValues = betabar.mapValues { variable => modelZsepDual.getValue(variable) }
          ReducedCostExpr.setConstant(-1.0 * alphaValue)

          for (j <- locationIndexes) {
            ReducedCostExpr.remove(xi(j))
            ReducedCostExpr.addTerm(-1.0 * betaValues(j), xi(j))
          }

          for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMomentMatrix(j1)(j2) > -EPS) {
            ReducedCostExpr.remove(lambda(j1, j2))
            ReducedCostExpr.addTerm(-1.0 * betabarValues(j1, j2), lambda(j1, j2))
          }

          ReducedCost.setExpr(ReducedCostExpr)

          //        pricingModel.addRangeFilter(-1E-6, Double.MaxValue, Array(ReducedCostExpr), Array(1.0))

          recordNow()
          if (generateIntitialScenario) {
            pricingModel.setParam(IloCplex.DoubleParam.TiLim, 1.0)
            pricingModel.setParam(IloCplex.DoubleParam.EpGap, 0.1)
          }
          pricingModel.solve()

          if (pricingModel.getStatus == IloCplex.Status.Optimal || (generateIntitialScenario && pricingModel.getStatus == IloCplex.Status.Feasible)) {
            print(s"Pricing problem is solved in ${timeCheckout()}s with status ${pricingModel.getStatus()} --- reduced costs = ${pricingModel.getObjValue()}".padTo(120, " ").mkString)
            if (pricingModel.getObjValue() > EPS) {
              recordNow()
              //            pricingModel.populate()
              val nSolutions = pricingModel.getSolnPoolNsolns()
              var scenariosToAdd = Seq.empty[Scenario]

              (0 until nSolutions).foreach(k => {
                if (pricingModel.getObjValue(k) > 0) {
                  val xiValues = xi.map { xi_i => pricingModel.getValue(xi_i, k) }
                  val failurePattern = locationIndexes.filter { j => xiValues(j) > 0.5 }
                  val scenario = new Scenario(failurePattern, 0)
                  if (!patternsInModelZsepDual.contains(scenario.failures)) {
                    scenariosToAdd = scenariosToAdd :+ scenario
                  }
                }
              })
              println(s"Scenarios added: ${scenariosToAdd.map { x => x.failures }.mkString(", ")}")
              if (scenariosToAdd.size == 0) {
                ret = None
              } else {
                ret = Option(scenariosToAdd)
              }
              //            println(s"Get solutions pool used time: ${timeCheckout()}s")

            } else {
              println("Pricing problem found no pattern with positive reduced cost.")
            }
          } else {
            throw new Exception(s"PricingModel ended up with status ${pricingModel.getStatus} when it should be solved to optimal.")
          }
          ret
        }
        recordNow()
        modelZsepDual.solve()

        if (generateIntitialScenario && timeFromRecorded("pricing") > 10) {
          println(" 10s limit reached for improving dummy solution, break")
          break
        }

        if (timeLimitReached()) {
          terminatedDueToTimeLimit = true
          println("Break loop since time limit is reached")
          break
        }

        //      println(modelZsepDual.getStatus)

        if (modelZsepDual.getStatus == IloCplex.Status.Optimal) {
          print(s"modelZsepDual is solved in ${timeCheckout()}s with status ${modelZsepDual.getStatus()} --- objective value = ${modelZsepDual.getObjValue()}".padTo(120, " ").mkString)
          currentObj = modelZsepDual.getObjValue

          //following lines control the absolute gap of pricing problem, such that the master problem stops when a target relative gap reached
          //        val reducedCostThreshold = currentObj * (gap + 1E-6) / SCALE_FACTOR
          //        pricingModel.setParam(IloCplex.DoubleParam.EpAGap, reducedCostThreshold)
          //        print(s"target pricing problem EpAGap: ${reducedCostThreshold}".padTo(60, " ").mkString)

          solvePricingProblem() match {
            case Some(scenarios) => {
              if (lastAddedPattern == scenarios.head.failures) {
                println("Duplicate pattern added. Terminate pricing.")
                break
              } else {
                lastAddedPattern = scenarios.head.failures
              }
              scenarios.foreach { scenario =>
                {
                  modelZsepDualAddCutForScenario(scenario)
                }
              }
            }
            case _ => {
              println()
              break
            }
          }

        } else {
          throw new Exception(s"modelZsepDual ended up with status ${modelZsepDual.getStatus} when it should be solved to optimal.")
        }

      }
    }

    if (!modelInfeasible && modelZsepDual.getStatus == IloCplex.Status.Optimal) {
      sepDual_objValue = modelZsepDual.getObjValue / SCALE_FACTOR
      (0 until Cuts.size).foreach(i => {
        scenarios(i).prob = modelZsepDual.getDual(Cuts(i)) / SCALE_FACTOR
      })

      if (generateIntitialScenario) {
        (0 until Cuts.size).foreach(i => {
          scenarios(i).prob = modelZsepDual.getDual(Cuts(i)) / SCALE_FACTOR
        })
        InitialScenarios = scenarios.filter { x => x.prob > 0 }.toArray

        InitialScenarios.foreach { x => println(s"${x.failures} with probability ${x.prob}") }
        println(s"End of generating initial scenarios assuming a solution with open locations ${openLocs}")
      }
    }

    pricingModel.end()
    modelZsepDual.end()

    if (modelInfeasible || terminatedDueToTimeLimit)
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