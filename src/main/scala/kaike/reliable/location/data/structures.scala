package kaike.reliable.location.data

import org.json4s.JsonAST.JArray
import org.json4s.JsonAST.JInt
import org.json4s.JsonAST.JDouble

case class Coordinate(val lat: Double, val lng: Double);

object GeoComputer {
  def distance(coord1: Coordinate, coord2: Coordinate): Double = {
        val x1 = Math.toRadians(coord1.lat);
        val y1 = Math.toRadians(coord1.lng);
        val x2 = Math.toRadians(coord2.lat);
        val y2 = Math.toRadians(coord2.lng);
        
        if(Math.abs(x1 - x2) < 10E-6 && Math.abs(y1 - y2) < 10E-6)
          return 0
        // great circle distance in radians
        var angle1 = Math.acos(Math.sin(x1) * Math.sin(x2)
                      + Math.cos(x1) * Math.cos(x2) * Math.cos(y1 - y2));

        // convert back to degrees
        angle1 = Math.toDegrees(angle1);

        // each degree on a great circle of Earth is 60 nautical miles
        60 * angle1;
        
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
  
  val failRate = candidateLocationIndexes.map { i => Math.min(1, 0.1 * beta * Math.exp(-(GeoComputer.distance(candidateLocations(i), newOrleans) / theta))) }
  
//  failrate.values.foreach { println  }
}

