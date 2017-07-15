package kaike.reliable.location.output

import kaike.reliable.location.data.LocationSolution
//import org.json4s._

import org.json4s.JsonDSL._
import org.json4s._
import org.json4s.native.JsonMethods._
import org.json4s.JsonAST.JValue
import org.json4s.JsonAST.JArray
import scalaj.http._
import kaike.reliable.location.model.RUFLPSolver
import kaike.reliable.location.model.CrossMomentSolver
import scala.util.control.NonFatal
import kaike.reliable.location.model.RobustUFLPSolver
import kaike.reliable.location.model.RUFLPSimpleCutSolver

object NetworkOutput {
  def jsonEncode(sol: LocationSolution, log: String) = {
    val demandPointsList = sol.instance.demandPoints.map { x => x.toJArray }.toList
    val openFacilitiesList = sol.openDCs.map { x => x.toJArray }.toList
    var json =
      ("numberofNodes" -> demandPointsList.size) ~
        ("numberofOpen" -> openFacilitiesList.size) ~
        ("solutionTime" -> sol.time) ~
        ("solver" -> sol.solver.SOLVER_NAME) ~
        ("problem" -> sol.instance.problemName) ~
        ("objectiveValue" -> sol.objValue) ~
        ("demandPoints" -> JArray(demandPointsList)) ~
        ("openFacilities" -> JArray(openFacilitiesList)) ~
        ("assignments" -> JArray(sol.assignments.toList.map(x => JArray(List(JInt(x._2.index), JInt(x._1.index)))))) ~
        ("gap" -> sol.gap * 100) ~
        ("status" -> sol.status)
        
    if(!log.equalsIgnoreCase(""))
      json = json ~ ("log" -> log)

    sol.solver match {
      case rculpSolver: RUFLPSolver => json = json ~ ("parameters" -> rculpSolver.instance.parameter.toString()) ~
        ("nbCuts" -> rculpSolver.nbCuts)
      case rculpSolver: CrossMomentSolver => json = json ~ ("parameters" -> rculpSolver.instance.parameter.toString()) ~
        ("nbCuts" -> rculpSolver.nbCuts)
      case rculpSolver: RobustUFLPSolver => json = json ~ ("parameters" -> rculpSolver.instance.parameter.toString()) ~
        ("nbCuts" -> rculpSolver.nbCuts)
      case rculpSolver: RUFLPSimpleCutSolver => json = json ~ ("parameters" -> rculpSolver.instance.parameter.toString()) ~
        ("nbCuts" -> rculpSolver.nbCuts)        
      case _ =>
    }

    compact(render(json))
  }

  def attemp(times: Int)(x: () => Any) = {
    (0 until times).takeWhile(i => {
      var ret = false
      try {
        println(x())
      } catch {
        case NonFatal(e) => {
          e.printStackTrace()
          ret = true
        }
      }
      ret
    })
  }
  
  def sendData(jsonData: String ) = {
    attemp(10)(
//      () => Http("http://location-solution-visualize.dev/api/solutions/").timeout(connTimeoutMs = 10000, readTimeoutMs = 20000).postForm(Seq("data" -> jsonData)).asString)
      () => Http("http://kaike.space/api/solutions/").timeout(connTimeoutMs = 10000, readTimeoutMs = 20000).postForm(Seq("data" -> jsonData)).asString)    
  }
  def post(sol: LocationSolution, log: String = "") = {
    sendData(jsonEncode(sol, log))
  }
  
  def postError(model:CrossMomentSolver, log:String = "") = {
   var json =
      ("numberofNodes" -> model.candidateDCs.size) ~
      ("solver" -> model.SOLVER_NAME) ~
      ("problem" -> model.instance.problemName) ~
      ("status" -> "Error") ~
      ("parameters" -> model.instance.parameter.toString())
    if(!log.equalsIgnoreCase(""))
      json = json ~ ("log" -> log)   
    sendData(compact(render(json)))  
  }
  
  def postError(model:RUFLPSolver) = {
   var json =
      ("numberofNodes" -> model.candidateDCs.size) ~
      ("solver" -> model.SOLVER_NAME) ~
      ("problem" -> model.instance.problemName) ~
      ("status" -> "Error") ~
      ("parameters" -> model.instance.parameter.toString()) 
    sendData(compact(render(json)))  
  }  
}