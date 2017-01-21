package kaike.reliable.location.data

import scala.io.Source
import scala.collection.mutable.ArrayBuffer


object InstanceReader {
  def readInstanceFrom(path: String):ProblemInstance = {
    val iter = Source.fromFile(path).getLines()
    val numberOfLocations = iter.next().trim.toInt
    val demands = ArrayBuffer.empty[DemandPoint]
    val candidateDCs = ArrayBuffer.empty[CandidateLocation]
    (1 to numberOfLocations).foreach(i=> {
      val numberStrings = iter.next().split(" +|\t")
      val index = numberStrings(0).toInt
      val demand = numberStrings(1).toDouble
      val fixedCost = numberStrings(3).toDouble
      val lat = numberStrings(4).toDouble
      val lng = - numberStrings(5).toDouble
      demands += new DemandPoint(index, demand, lat, lng)
      candidateDCs += new CandidateLocation(index, fixedCost, lat, lng)
      
    })
    
    ProblemInstance( demandPoints = demands.toSeq,  candidateLocations = candidateDCs.toSeq)
  }
}