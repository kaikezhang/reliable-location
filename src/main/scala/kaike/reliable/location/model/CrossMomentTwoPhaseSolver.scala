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
import scala.reflect.internal.util.TableDef.Column

class CrossMomentTwoPhaseSolver(override val instance: CrossMomentProblemInstance, override val instructor: SolverInstructor) extends CrossMomentSolverAbstract(instance, instructor, "CuttingPlane + Two phase CG") {

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
              println("  fail: Seperate solution with column generation")
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

  def SEPERATE(openValues: IndexedSeq[Boolean], generateIntitialScenario:Boolean = false): (Double, Seq[Scenario]) = {

    var terminatedDueToTimeLimit = false

    var sepDual_objValue = Double.MaxValue

    val openLocs = TreeSet(locationIndexes.filter { j => openValues(j) }:_*)
    
    val patternsInModelZsepDual = collection.mutable.HashSet.empty[Set[Int]]

     
    val scenarios = collection.mutable.ArrayBuffer.empty[Scenario]

    val modelZsep = new IloCplex()

    val alpha = modelZsep.addRange(SCALE_FACTOR, SCALE_FACTOR)
    val beta = locationIndexes.map { j => modelZsep.addRange(crossMomentMatrix(j)(j), crossMomentMatrix(j)(j)) }
    val betabar = (for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMomentMatrix(j1)(j2) > -EPS)
      yield ((j1, j2), modelZsep.addRange(crossMomentMatrix(j1)(j2), crossMomentMatrix(j1)(j2)))).toMap
    


//    modelZsepDual.setParam(IloCplex.IntParam.RootAlg, IloCplex.Algorithm.Primal)
    modelZsep.setOut(null)
    //Turn off preprocessing otherwise the model will return status of infeasibleorunbounded
    modelZsep.setParam(IloCplex.BooleanParam.PreInd, false)
   
    val ObjectiveExpr = modelZsep.addMaximize();
    
    val Columns = collection.mutable.ArrayBuffer.empty[IloNumVar]

    def modelZsepAddColumnForScenario(scenario: Scenario, firstPhase:Boolean = false) = {
      var column = firstPhase match {
        case true => modelZsep.column(ObjectiveExpr, 0)
        case false => modelZsep.column(ObjectiveExpr, getTransptCostsForScenario(openLocs, scenario))
      }
      
      column = column.and(modelZsep.column(alpha, 1.0))
      
      scenario.failures.foreach { k => column = column.and(modelZsep.column(beta(k), 1.0)) }
      
      for (j1 <- scenario.failures; j2 <- scenario.failures if j1 < j2 && crossMomentMatrix(j1)(j2) > -EPS) {
        column = column.and(modelZsep.column(betabar(j1, j2), 1.0))
      }

      val column_var = modelZsep.numVar(column, 0, SCALE_FACTOR)
      
      Columns += column_var
      scenarios += scenario
      patternsInModelZsepDual += scenario.failures
    }

    if(generateIntitialScenario){
      ArtificialSeedRealizations.foreach { scenario =>
        modelZsepAddColumnForScenario(scenario, true)
      }      
    } else {
      InitialScenarios.foreach { scenario =>
        modelZsepAddColumnForScenario(scenario)
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
      val pi_eff = if(openLocs.contains(j)) -distance(i)(j) * demands(i).demand else -demands(i).emergencyCost * demands(i).demand
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

    val ReducedCostLinearExpr = pricingModel.linearNumExpr()
    val ReducedCost = pricingModel.addMaximize(ReducedCostLinearExpr)

    demandIndexes.foreach { i => ReducedCostLinearExpr.addTerm(1.0, pi(i)) }

    if (generateIntitialScenario) {
      
      // create artificial variables
      var column_alpha = modelZsep.column(ObjectiveExpr, -1.0)
      column_alpha = column_alpha.and(modelZsep.column(alpha, 1));     
      val art_var_alpha = modelZsep.numVar(column_alpha, 0, Double.MaxValue)
      
      val column_beta = locationIndexes.toArray.map { j => modelZsep.column(ObjectiveExpr, -1.0) }
      locationIndexes.foreach { j => column_beta(j) =  column_beta(j).and(modelZsep.column(beta(j), 1))}   
      val var_beta = locationIndexes.map { j => modelZsep.numVar(column_beta(j), 0, Double.MaxValue) }
      
      var column_betabar = (for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMomentMatrix(j1)(j2) > -EPS)
        yield ((j1, j2), modelZsep.column(ObjectiveExpr, -1.0))).toMap
      column_betabar =  column_betabar.map { case ((j1,j2), column) =>  ((j1,j2), column.and(modelZsep.column(betabar(j1, j2), 1)))  }
      val var_betabar = column_betabar.map { case ((j1,j2), column) =>  ((j1,j2), modelZsep.numVar(column, 0, Double.MaxValue ))  }
            
    
      val firstPhasePricingModel = new IloCplex()
      val xi = locationIndexes.map { j => firstPhasePricingModel.boolVar() }
      val lambda = (for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMomentMatrix(j1)(j2) > -EPS)
      yield ((j1, j2), firstPhasePricingModel.numVar(0, 1))).toMap

      for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMomentMatrix(j1)(j2) > -EPS) {
        firstPhasePricingModel.addLe(lambda(j1, j2), xi(j1))
        firstPhasePricingModel.addLe(lambda(j1, j2), xi(j2))
        firstPhasePricingModel.addGe(lambda(j1, j2), firstPhasePricingModel.sum(-1.0, firstPhasePricingModel.sum(xi(j1), xi(j2))))
      }

      firstPhasePricingModel.setOut(null)
      firstPhasePricingModel.setWarning(null)
      firstPhasePricingModel.setParam(IloCplex.IntParam.SolnPoolCapacity, 10)
      firstPhasePricingModel.setParam(IloCplex.IntParam.PopulateLim, 100)
      
      val ReducedCost = firstPhasePricingModel.addMaximize() 
      
      def generateScenarioReduceInfeasibility(): Option[Seq[Scenario]] = {
        var ret: Option[Seq[Scenario]] = None
        val alphaDual = modelZsep.getDual(alpha)
        val betaDuals = beta.map { beta_i => modelZsep.getDual(beta_i) }
        val betabarDuals = betabar.mapValues { betabar_j1j2 => modelZsep.getDual(betabar_j1j2) }

        val ReducedCostLinearExpr = firstPhasePricingModel.linearNumExpr()
        
        ReducedCostLinearExpr.setConstant(-1.0 * alphaDual)

        for (j <- locationIndexes) {
          ReducedCostLinearExpr.addTerm(-1.0 * betaDuals(j), xi(j))
        }

        for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMomentMatrix(j1)(j2) > -EPS) {
          ReducedCostLinearExpr.addTerm(-1.0 * betabarDuals(j1, j2), lambda(j1, j2))
        }

        ReducedCost.setExpr(ReducedCostLinearExpr)
        
        recordNow()
                
        firstPhasePricingModel.solve()

        if (firstPhasePricingModel.getStatus == IloCplex.Status.Optimal
            || firstPhasePricingModel.getStatus == IloCplex.Status.Feasible ) {
          print(s"First phase pricing problem is solved in ${timeCheckout()}s with status ${firstPhasePricingModel.getStatus()} --- reduced costs = ${firstPhasePricingModel.getObjValue()}".padTo(120, " ").mkString)          
          if (firstPhasePricingModel.getObjValue() > EPS) {
            recordNow()

            val nSolutions = firstPhasePricingModel.getSolnPoolNsolns()
            var scenariosToAdd = Seq.empty[Scenario]
           
            (0 until nSolutions).foreach(k => {
              if(firstPhasePricingModel.getObjValue(k) > 0){
                val xiValues = xi.map { xi_i => firstPhasePricingModel.getValue(xi_i, k) }
                val failurePattern = locationIndexes.filter { j => xiValues(j) > 0.5 }
                val scenario = new Scenario(failurePattern, 0)               
                if(! patternsInModelZsepDual.contains(scenario.failures)){
                  scenariosToAdd = scenariosToAdd :+ scenario
                }              
              }
            })
            
            println(s"Scenarios added: ${scenariosToAdd.map { x => x.failures }.mkString(", ")}")
            if(scenariosToAdd.size == 0){            
              ret = None
            } else {
              ret = Option(scenariosToAdd)
            }
            
          } else {
            println("Pricing problem found no pattern with positive reduced cost.")
          }
        } else {
          throw new Exception(s"PricingModel ended up with status ${firstPhasePricingModel.getStatus} when it should be solved to optimal.")
        }
        ret
      }

      println("Generating feasible scenarios:")

      breakable { while (true) {
        recordNow()
        modelZsep.solve()
        print(s"Model Zsep first phase solved in ${timeCheckout()}s with status ${modelZsep.getStatus} obj: ${modelZsep.getObjValue}".padTo(120, " ").mkString)
        if (modelZsep.getStatus == IloCplex.Status.Optimal && modelZsep.getObjValue > -EPS) {
          break
        } else if (modelZsep.getStatus == IloCplex.Status.Optimal && modelZsep.getObjValue < EPS) {
          generateScenarioReduceInfeasibility() match {
            case Some(scenarios) => {
              scenarios.foreach { scenario => {
                modelZsepAddColumnForScenario(scenario, true)
                }
              }              
            }
            case _ => {
              modelInfeasible = true
              break
            }
          }
           
        } else {
          throw new Exception(s"PricingModel ended up with status ${firstPhasePricingModel.getStatus} when it should be solved to optimal.")
        }

        if (timeLimitReached()) {
          terminatedDueToTimeLimit = true
          break
        }
      }}
      println("End of generating feasible scenarios")
      println()
      firstPhasePricingModel.end()
    }
    
    if(generateIntitialScenario){
      (0 until Columns.size).foreach(i => {
        scenarios(i).prob = modelZsep.getValue(Columns(i)) / SCALE_FACTOR
      })
      InitialScenarios = scenarios.filter { x => x.prob > 0 }.toArray
      
      InitialScenarios.foreach { x => println(s"${x.failures} with probability ${x.prob}") }
      println()
      
      return (0, InitialScenarios.filter { p => p.prob > 0 })
    }
    
//    println(modelZsepDual.getStatus)


    var lastAddedPattern = Set.empty[Int]
    var currentObj = 0.0
    
    pricingModel.setParam(IloCplex.IntParam.SolnPoolCapacity, 10)
    pricingModel.setParam(IloCplex.IntParam.PopulateLim, 100)
    
    
    recordNow("pricing")
    
    if(!modelInfeasible) {
    breakable { while (true) {
      
      def solvePricingProblem(): Option[Seq[Scenario]] = {
        var ret: Option[Seq[Scenario]] = None
        val alphaDual = modelZsep.getDual(alpha)
        val betaDuals = beta.map { beta_i => modelZsep.getDual(beta_i) }
        val betabarDuals = betabar.mapValues { variable => modelZsep.getDual(variable) }
        ReducedCostLinearExpr.setConstant(-1.0 * alphaDual)

        for (j <- locationIndexes) {
          ReducedCostLinearExpr.remove(xi(j))
          ReducedCostLinearExpr.addTerm(-1.0 * betaDuals(j), xi(j))
        }

        for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMomentMatrix(j1)(j2) > -EPS) {
          ReducedCostLinearExpr.remove(lambda(j1, j2))
          ReducedCostLinearExpr.addTerm(-1.0 * betabarDuals(j1, j2), lambda(j1, j2))
        }

        ReducedCost.setExpr(ReducedCostLinearExpr)
        
//        pricingModel.addRangeFilter(-1E-6, Double.MaxValue, Array(ReducedCostExpr), Array(1.0))
        
        recordNow()

        pricingModel.solve()

        if (pricingModel.getStatus == IloCplex.Status.Optimal || pricingModel.getStatus == IloCplex.Status.Feasible ) {
          print(s"Pricing problem is solved in ${timeCheckout()}s with status ${pricingModel.getStatus()} --- reduced costs = ${pricingModel.getObjValue()}".padTo(120, " ").mkString)          
          if (pricingModel.getObjValue() > EPS) {
            recordNow()
//            pricingModel.populate()
            val nSolutions = pricingModel.getSolnPoolNsolns()
            var scenariosToAdd = Seq.empty[Scenario]
           
            (0 until nSolutions).foreach(k => {
              if(pricingModel.getObjValue(k) > 0){
                val xiValues = xi.map { xi_i => pricingModel.getValue(xi_i, k) }
                val failurePattern = locationIndexes.filter { j => xiValues(j) > 0.5 }
                val scenario = new Scenario(failurePattern, 0)               
                if(! patternsInModelZsepDual.contains(scenario.failures)){
                  scenariosToAdd = scenariosToAdd :+ scenario
                }              
              }
            })
            println(s"Scenarios added: ${scenariosToAdd.map { x => x.failures }.mkString(", ")}")
            if(scenariosToAdd.size == 0){            
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
      modelZsep.solve()
      
      if(generateIntitialScenario && timeFromRecorded("pricing") > 10) {
        println(" 10s limit reached for improving dummy solution, break")
        break
      }
      
      if (timeLimitReached()) {
        terminatedDueToTimeLimit = true
        println("Break loop since time limit is reached")
        break
      }      
      
//      println(modelZsepDual.getStatus)
      
      if(modelZsep.getStatus == IloCplex.Status.Optimal){
        print(s"modelZsepDual is solved in ${timeCheckout()}s with status ${modelZsep.getStatus()} --- objective value = ${modelZsep.getObjValue()}".padTo(120, " ").mkString)         
        currentObj = modelZsep.getObjValue
                
        solvePricingProblem() match {
          case Some(scenarios) => {
            if(lastAddedPattern == scenarios.head.failures){
              println("Duplicate pattern added. Terminate pricing.")
              break
            } else {
              lastAddedPattern = scenarios.head.failures
            }
            scenarios.foreach { scenario => {
              modelZsepAddColumnForScenario(scenario)
              }
            }
          }
          case _ => {
            println()
            break
          }
        }

      } else {
        throw new Exception(s"modelZ ended up with status ${modelZsep.getStatus} when it should be solved to optimal.")
      }

    }}}


    if (!modelInfeasible && modelZsep.getStatus == IloCplex.Status.Optimal) {
      sepDual_objValue = modelZsep.getObjValue / SCALE_FACTOR
      (0 until Columns.size).foreach(i => {
        scenarios(i).prob = modelZsep.getValue(Columns(i)) / SCALE_FACTOR
      })
      
    }

    
    pricingModel.end()
    modelZsep.end()
    
    

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