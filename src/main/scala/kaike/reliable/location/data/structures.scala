package kaike.reliable.location.data

import org.json4s.JsonAST.JArray
import org.json4s.JsonAST.JInt
import org.json4s.JsonAST.JDouble

case class Coordinate(val lat: Double, val lng: Double);

object GeoComputer {
  def distance(coord1: Coordinate, coord2: Coordinate): Double = {    
    def deg2rad( deg: Double) = { (deg * Math.PI / 180); }
    
    def rad2deg(rad : Double) = { (rad * 180 / Math.PI); }

    val lat1 = coord1.lat;
    val lng1 = coord1.lng;
    val lat2 = coord2.lat;
    val lng2 = coord2.lng;    

    var theta = 0.0; var dist = 0.0;
    theta = lng1 - lng2;
    val deta_lon = lng1 - lng2;
    val deta_lat = lat1 - lat2;
  
    if (Math.abs(deta_lon) + Math.abs(deta_lat) < 0.001) {
      return 0.0;
    }
  
    dist = Math.sin(deg2rad(lat1)) * Math.sin(deg2rad(lat2)) +
           Math.cos(deg2rad(lat1)) * Math.cos(deg2rad(lat2)) * Math.cos(deg2rad(theta));
    dist = Math.acos(dist);
    dist = rad2deg(dist);
    dist = dist * 60 * 1.1515;
    dist;    
        
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

