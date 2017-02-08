package kaike.reliable.location.data

import org.json4s.JsonAST.JArray
import org.json4s.JsonAST.JInt
import org.json4s.JsonAST.JDouble

case class Coordinate(val lat: Double, val lng: Double);

object GeoComputer {
  def distance(coord1: Coordinate, coord2: Coordinate): Double = {    

    val lat1 = Math.toRadians(coord1.lat);
    val lng1 = Math.toRadians(coord1.lng);
    val lat2 = Math.toRadians(coord2.lat);
    val lng2 = Math.toRadians(coord2.lng);
    val theta = lng1 - lng2;

    var dist = 0.0;
    
    val deta_lng = lng1 - lng2;
    val deta_lat = lat1 - lat2;
  
    if (Math.abs(deta_lng) + Math.abs(deta_lat) < 1E-6) {
      return 0.0;
    }
  
    dist = Math.sin(lat1) * Math.sin(lat2) +
           Math.cos(lat1) * Math.cos(lat2) * Math.cos(theta);
    dist = Math.acos(dist);
    dist = dist.toDegrees;
    dist * 60 * 1.1515;    
        
  }
}

class DemandPoint(val index: Int,  val demand: Double, val emergencyCost:Double, override val lat: Double, override val lng: Double) extends Coordinate(lat, lng){
  def toJArray() = JArray(List(JInt(index), JDouble(lat), JDouble(lng) , JDouble(demand)))
}

class CandidateLocation(val index: Int, val fixedCosts: Double,  override val lat: Double, override val lng: Double) extends Coordinate(lat, lng){
  def toJArray() = JArray(List(JInt(index), JDouble(lat), JDouble(lng)))
}

case class ReliableLocationParameter(alpha:Double = 1, theta:Int = 400){
  override def toString() = {
    s"Alpha = ${"%.2f".format(alpha)}\nTheta = ${theta}"
  }
}

case class CrossMonmentParameter(alpha: Double = 1, theta:Int = 400) {
  override def toString() = {
    s"Alpha = ${"%.2f".format(alpha)}\nTheta = ${theta}"
  }  
}

class ProblemInstance(val demandPoints: IndexedSeq[DemandPoint],  val candidateLocations: IndexedSeq[CandidateLocation], val problemName:String = "UFLP") {
  val demandsPointIndexes = 0 until demandPoints.size
  val candidateLocationIndexes = 0 until candidateLocations.size  
  
  // compute distance matrix in miles
  val distance = Array.tabulate(demandPoints.length, candidateLocations.length)((i,j) => {
    GeoComputer.distance(demandPoints(i), candidateLocations(j))
  })
}

abstract class ReliableProblemInstance( demandPoints: IndexedSeq[DemandPoint],  
                                        candidateLocations: IndexedSeq[CandidateLocation],
                                        problemName:String) 
                                                          extends ProblemInstance(demandPoints, candidateLocations, problemName){
  val failRate:IndexedSeq[Double]
}

case class StochasticReliableLocationProblemInstance( override val demandPoints: IndexedSeq[DemandPoint],  
                                            override val candidateLocations: IndexedSeq[CandidateLocation],
                                            parameter: ReliableLocationParameter = ReliableLocationParameter()) 
                                                          extends ReliableProblemInstance(demandPoints, candidateLocations, "Stochastic RUFLP"){

  val alpha = parameter.alpha
  val theta = parameter.theta
  
  val newOrleans = Coordinate(30.07, -89.93)
  val failRate = candidateLocationIndexes.map { i => Math.min(1, 0.01 + 0.1 * alpha * Math.exp(-(GeoComputer.distance(candidateLocations(i), newOrleans) / theta))) }

}

case class RobustReliableLocationProblemInstance( override val demandPoints: IndexedSeq[DemandPoint],  
                                          override val candidateLocations: IndexedSeq[CandidateLocation], 
                                          parameter: ReliableLocationParameter = ReliableLocationParameter())
                                                          extends ReliableProblemInstance(demandPoints, candidateLocations, "Robust RUFLP Marginal"){
  
  val alpha = parameter.alpha
  val theta = parameter.theta
  
  val newOrleans = Coordinate(30.07, -89.93)
  val failRate = candidateLocationIndexes.map { i => Math.min(1,  alpha * Math.exp(-(GeoComputer.distance(candidateLocations(i), newOrleans) / theta))) }

}

class Scenario(val failures:Set[Int], var prob:Double)
  
case class CrossMonmentProblemInstance(override val demandPoints: IndexedSeq[DemandPoint],
                                       override val candidateLocations: IndexedSeq[CandidateLocation], 
                                       parameter: CrossMonmentParameter = CrossMonmentParameter())
                                                          extends ProblemInstance(demandPoints, candidateLocations, "Robust RUFLP Crossmonment") {
  val alpha = parameter.alpha
  val theta = parameter.theta
  val newOrleans = Coordinate(30.07, -89.93)
  val failRate = candidateLocationIndexes.map { i => Math.min(1, alpha * Math.exp(-(GeoComputer.distance(candidateLocations(i), newOrleans) / theta))) }
  
  private val candidateDistance = Array.tabulate(candidateLocations.length, candidateLocations.length)((i,j) => {
    GeoComputer.distance(candidateLocations(i), candidateLocations(j))
  })

  private val nearestLoc = candidateLocationIndexes.map(i => candidateLocationIndexes.filter(j => j != i).minBy(j => candidateDistance(i)(j)) )
  
  val crossMonmentMatrix = Array.tabulate(candidateLocations.size, candidateLocations.size)((i, j) => {
    if(i == j)
      failRate(i)
    else if(j == nearestLoc(i) || i == nearestLoc(j)){
      failRate(i) * failRate(j) // / alpha
//      -1
    } else
      -1
  })
  
  crossMonmentMatrix.foreach { x => println(x.mkString("[", ", ", "]")) }
  
  val nbRealizations = candidateLocations.size
  
  def generateRandomScenario(): Scenario = {
    val failures = candidateLocationIndexes.filter { j => Math.random() > 0.5 }.toSet
    new Scenario(failures, 0)
  }
  
  def generateSingletonScenario(i: Int): Scenario = {
    val failures = candidateLocationIndexes.filter { j => j == i }.toSet
    new Scenario(failures, 0)    
  }
  
  val realizations = (0 until nbRealizations).map(i => 
//    generateSingletonScenario(i)
    generateRandomScenario()
  )
  
}


