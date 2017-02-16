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
import kaike.reliable.location.data.CrossMomentProblemInstance
import scala.util.control.Breaks._
import ilog.concert.IloRange
import kaike.reliable.location.data.Scenario

class CrossMomentSolver(override val instance: CrossMomentProblemInstance, override val instructor: SolverInstructor) extends SolverCommon(instance, instructor, "CuttingPlane + ColumnGen") {

  val SCALE_FACTOR = 1E6
  
  val crossMomentMatrix = instance.crossMomentMatrix.map { x => x.map { x => if(x > 0 ) x * SCALE_FACTOR  else x} }

  println(s"Cross moment matrix with scale factor = ${SCALE_FACTOR}:")
  crossMomentMatrix.foreach { x => println(x.map { x => x.toString.padTo(25, " ").mkString }.mkString("[", ", ", "]")) }
  println()

  val randomRealizations = instance.realizations
  
  var modelInfeasible = false

  var nbCuts = 0

  var upperBound = Double.MaxValue
  var lowerBound = 0.0

  var InitialScenarios = Array.empty[Scenario]



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
      
      breakable { while (!modelInfeasible) {

        cuttingPlaneMainProblem.setParam(IloCplex.DoubleParam.TiLim, timeLeft())

        recordNow()
        if (cuttingPlaneMainProblem.solve()) {
          val openValues = locationIndexes.map { j => cuttingPlaneMainProblem.getValue(open(j)) > 0.5 }
          val openLocs = locationIndexes.filter { j => openValues(j) }
          val setupCosts = openLocs.map { j => candidateDCs(j).fixedCosts }.sum
          val phiValue = cuttingPlaneMainProblem.getValue(phi)
          
          println(s"Cutting plane master problem -- obj:${cuttingPlaneMainProblem.getObjValue} phi:${phiValue} in ${timeCheckout()}s")
          
          recordNow("Seperation")
          val (trspCosts, scenarios) = SEPERATE(openValues)  
          println(s"Seperation problem is solved in ${timeCheckout("Seperation")}s --- transportation costs: ${trspCosts} for solution ${openLocs}")

          if (scenarios.size == 0) {
            println("Time limit is reached during solving seperation problem.")
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

          var cut = cuttingPlaneMainProblem.linearNumExpr(trspCosts)

          for (j <- locationIndexes if !openValues(j)) {
            val newOpenLocs = openLocs :+ j
            val incrementalCost = getTransptCostsForScenarios(newOpenLocs.toSet, scenarios) - trspCosts
            cut.addTerm(incrementalCost, open(j))
          }
          cuttingPlaneMainProblem.addGe(phi, cut)
          nbCuts += 1
          println(s"Number of Cuts (for cutting plane master problem) added: ${nbCuts} ---------------------------------Time ellasped ${timeUsed}s")

        } else {
          println("Should not come here Error 003")
          println(s"Cutting Plane Master Problem has status ${cuttingPlaneMainProblem.getStatus}")
          break
        }

      }}
      
      if(!modelInfeasible){   
        var finalGap = (upperBound - lowerBound) / lowerBound
        if(finalGap < 0) finalGap = 0.0
      
        
        val assignments = demandIndexes.map { i => {
            (demands(i), candidateDCs(openDCs.minBy { j => distance(i)(j) }) )
          } }.toSeq        
        val status = if( timeLimitReached()) "Time Reached" else  "Gap Reached"
          
        println(s"Final Upper bound: ${upperBound} -- Lower bound: ${lowerBound} --- gap:${finalGap} ----status:${status} ----Time used:${timeUsed}")
        ret = Some(LocationSolution(instance = instance, openDCs = openDCs.map { j => candidateDCs(j) }, assignments = assignments,
          time = timeUsed, solver = this, objValue = Math.round(upperBound), finalGap, status = status))
      } else {
        nbCuts = 0
        println("Cross moment matrix is infeasible")
        ret = Some(LocationSolution(instance = instance, openDCs = Seq.empty , assignments = Seq.empty,
          time = timeUsed, solver = this, objValue = 0, gap = 0, status = "Infeasible") )       
      }

    } catch {
      case e: CpxException => {
        println("Cplex exception caught: " + e);
        e.printStackTrace(Console.out)
      }
      case NonFatal(e)     => {
        println("exception caught: " + e);
        e.printStackTrace(Console.out)
      }
    } finally {
      cuttingPlaneMainProblem.end()
    }
    ret
  }
  
  def generateInitialScenarios() = {
    SEPERATE(openValues = locationIndexes.map(i => true), generateIntitialScenario = true)
  }

  def SEPERATE(openValues: IndexedSeq[Boolean], generateIntitialScenario:Boolean = false): (Double, Seq[Scenario]) = {

    var terminatedDueToTimeLimit = false

    var sepDual_objValue = Double.MaxValue

    val openLocs = TreeSet(locationIndexes.filter { j => openValues(j) }:_*)

    if(generateIntitialScenario)
      println(s"Generating initial scenarios assuming a solution with open locations ${openLocs}")
    else
      println(s"Solving seperation problem for ${openLocs}")

    val scenarios = collection.mutable.ArrayBuffer.empty[Scenario]

    val modelZsepDual = new IloCplex()

    val alpha = modelZsepDual.numVar(-Double.MaxValue, Double.MaxValue)
    val beta = locationIndexes.map { j => modelZsepDual.numVar(-Double.MaxValue, Double.MaxValue) }
    val betabar = (for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMomentMatrix(j1)(j2) > 0)
      yield ((j1, j2), modelZsepDual.numVar(-Double.MaxValue, Double.MaxValue))).toMap
    
    val objCosts = modelZsepDual.linearNumExpr()
    objCosts.addTerm(SCALE_FACTOR, alpha)

    locationIndexes.foreach { j => objCosts.addTerm(crossMomentMatrix(j)(j), beta(j)) }

    for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMomentMatrix(j1)(j2) > 0 ) {
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
      
      for (j1 <- scenario.failures; j2 <- scenario.failures if j1 < j2 && crossMomentMatrix(j1)(j2) > 0) {
        cutlhs.addTerm(1.0, betabar(j1, j2))
      }

      val cut = modelZsepDual.addGe(cutlhs, getTransptCostsForScenario(openLocs, scenario))
      Cuts += cut
      scenarios += scenario
    }

    if(generateIntitialScenario){
      randomRealizations.foreach { scenario =>
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
    val lambda = (for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMomentMatrix(j1)(j2) > 0)
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

    for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMomentMatrix(j1)(j2) > 0) {
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
      val lambda = (for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMomentMatrix(j1)(j2) > 0)
      yield ((j1, j2), unboundedModel.numVar(0, 1))).toMap

      for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMomentMatrix(j1)(j2) > 0) {
        unboundedModel.addLe(lambda(j1, j2), xi(j1))
        unboundedModel.addLe(lambda(j1, j2), xi(j2))
        unboundedModel.addGe(lambda(j1, j2), unboundedModel.sum(-1.0, unboundedModel.sum(xi(j1), xi(j2))))
      }
      val unboundedReducedCostLinearExpr = unboundedModel.linearNumExpr()
      val unboundedReducedCost = unboundedModel.addMinimize(unboundedReducedCostLinearExpr)
      unboundedModel.setOut(null)
      unboundedModel.setWarning(null)
      
      def generateInfeasibilityCut(): Option[Scenario] = {
        var ret:Option[Scenario] = None
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
        for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMomentMatrix(j1)(j2) > 0) {
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
          println("Should not come here erro xex1")
          println(s"Pricing problem for unbound sepDual solved in ${timeCheckout()}s with status ${unboundedModel.getStatus()}")
        }
        ret
      }

      println("Generating feasible scenarios")

      breakable { while (true) {
        recordNow()
        modelZsepDual.solve()
        println(s"Model ZsepDual solved in ${timeCheckout()}s with status ${modelZsepDual.getStatus}")
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
          println("Should not come here: Erro 001")
          println(s"Model ZsepDual solved in ${timeCheckout()}s with status ${modelZsepDual.getStatus()}")
        }

        if (timeLimitReached()) {
          terminatedDueToTimeLimit = true
          break
        }
      }}
      println("End of generating feasible scenarios")
      unboundedModel.end()
    }
    
//    println(modelZsepDual.getStatus)

    var lastObjValueModelZsepDual = 0.0
    var noImproveCount = 0
    breakable { while (!modelInfeasible) {
      
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

        for (j1 <- locationIndexes; j2 <- locationIndexes if j1 < j2 && crossMomentMatrix(j1)(j2) > 0) {
          ReducedCostExpr.remove(lambda(j1, j2))
          ReducedCostExpr.addTerm(-1.0 * betabarValues(j1, j2), lambda(j1, j2))
        }

        ReducedCost.setExpr(ReducedCostExpr)
        
        recordNow()
        pricingModel.solve()

        if (pricingModel.getStatus == IloCplex.Status.Optimal) {
          println(s"Pricing problem is solved in ${timeCheckout()}s with status ${pricingModel.getStatus()} --- reduced costs = ${pricingModel.getObjValue()}")          
          if (pricingModel.getObjValue() > EPS) {
            val xiValues = xi.map { xi_i => pricingModel.getValue(xi_i) }
            val failurePattern = locationIndexes.filter { j => xiValues(j) > 0.5 }
            ret = Option(new Scenario(failurePattern, 0.0))
          } else {
            println("Pricing problem found no pattern with positive reduced cost.")
          }
        } else {
          println("Should not come here Error 01TX")
          println(s"PricingModel status: ${pricingModel.getStatus}")
        }
        ret
      }
      
      modelZsepDual.solve()
      
      if (timeLimitReached()) {
        terminatedDueToTimeLimit = true
        println("Break loop since time limit is reached")
        break
      }      
      
//      println(modelZsepDual.getStatus)
      
      if(modelZsepDual.getStatus == IloCplex.Status.Optimal){
        print(s"modelZsepDual is solved in ${timeCheckout()}s with status ${modelZsepDual.getStatus()} --- objective value = ${modelZsepDual.getObjValue()}".padTo(120, " ").mkString)         
        val currentObj = modelZsepDual.getObjValue()
        solvePricingProblem() match {
          case Some(scenario) => {
            modelZsepDualAddCutForScenario(scenario)
          }
          case _ => {
            println()
            break
          }
        }
        if(currentObj - lastObjValueModelZsepDual > 0.01 * SCALE_FACTOR ){ // improve robustness, otherwise the algorithm may be stuck at some iteration        
           noImproveCount = 0
        } else {
          noImproveCount += 1
          if(noImproveCount > 5){
            println()
            break
          }
        }
        lastObjValueModelZsepDual = currentObj
      } else {
        println("Should not come here Error x2tv")
        println("ModelZsepDual status: " + modelZsepDual.getStatus)
        break
      }

    }}

    if (!modelInfeasible && modelZsepDual.getStatus == IloCplex.Status.Optimal) {
      sepDual_objValue = modelZsepDual.getObjValue / SCALE_FACTOR
      (0 until Cuts.size).foreach(i => {
        scenarios(i).prob = modelZsepDual.getDual(Cuts(i)) / SCALE_FACTOR
      })
      
      if(generateIntitialScenario){
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