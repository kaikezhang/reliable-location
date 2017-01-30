package kaike.reliable.location.output

import kaike.reliable.location.data.LocationSolution
//import org.json4s._

import org.json4s.JsonDSL._
import org.json4s._
import org.json4s.native.JsonMethods._
import org.json4s.JsonAST.JValue
import org.json4s.JsonAST.JArray
import scalaj.http._
import kaike.reliable.location.model.RUCFLSolver

object Visualizer {
  def jsonEncode(sol: LocationSolution) = {
    val demandPointsList = sol.instance.demandPoints.map { x => x.toJArray }.toList
    val openFacilitiesList = sol.openDCs.map { x => x.toJArray }.toList
    var json = 
               ("numberofNodes" -> demandPointsList.size) ~
               ("numberofOpen" -> openFacilitiesList.size) ~
               ("solutionTime" -> sol.time) ~
               ("solver" -> sol.solver.SOLVER_NAME) ~
               ("objectiveValue" -> sol.objValue) ~
               ("demandPoints" -> JArray(demandPointsList))  ~ 
               ("openFacilities" ->  JArray(openFacilitiesList)) ~ 
               ("assignments" -> JArray(sol.assignments.toList.map(x => JArray( List(JInt(x._2.index), JInt(x._1.index) )))))
               
    sol.solver match {
      case rculpSolver: RUCFLSolver => json = json ~ ("parameters" -> sol.instance.parameter.toString()) ~
                                              ("nbCuts" -> rculpSolver.nbCuts)
      case _ =>
    }
      
    
    compact(render(json))
  }
  def post(sol: LocationSolution) = {
    val jsonData = jsonEncode(sol)
    println(jsonData)
    Http("http://location-solution-visualize.dev/api/solutions/").postForm(Seq("data" -> jsonData)).asString
    
  }
}