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

case class Parameter(alpha:Double = 1, theta:Double = 400){
  override def toString() = {
    s"Alpha = ${alpha}\nTheta = ${theta}"
  }
}

case class ProblemInstance( demandPoints: IndexedSeq[DemandPoint],  candidateLocations: IndexedSeq[CandidateLocation], parameter: Parameter = Parameter()){
  val demandsPointIndexes = 0 until demandPoints.size
  val candidateLocationIndexes = 0 until candidateLocations.size
  
  val beta = parameter.alpha
  val theta = parameter.theta
  
  val newOrleans = Coordinate(30.07, -89.93)
  
  // compute distance matrix in miles
  val distance = Array.ofDim[Double](demandPoints.length, candidateLocations.length)
  for(i <- demandsPointIndexes; j <- candidateLocationIndexes){
    distance(i)(j)  = GeoComputer.distance(demandPoints(i), candidateLocations(j))
  }
  
//  val distanceMap = (for(demand <- demandPoints; dc <- candidateLocations) yield ((demand,dc), GeoComputer.distance(demand, dc))) toMap
  
//  val failrate = candidateLocations.map { location => {
//    (location, Math.min(1, 0.1 * beta * Math.exp(-(GeoComputer.distance(location, newOrleans) / theta))))
//  } }.toMap
  
  val failRate = candidateLocationIndexes.map { i => Math.min(1, 0.01 + 0.1 * beta * Math.exp(-(GeoComputer.distance(candidateLocations(i), newOrleans) / theta))) }
  
//  failrate.values.foreach { println  }
}

