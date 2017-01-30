package kaike.reliable.location.data

import scala.io.Source
import scala.collection.mutable.ArrayBuffer


object InstanceReader {
  def readInstanceFrom(path: String): (IndexedSeq[DemandPoint], IndexedSeq[CandidateLocation]) = {
    val iter = Source.fromFile(path).getLines()
    val numberOfLocations = iter.next().trim.toInt
    val demands = ArrayBuffer.empty[DemandPoint]
    val candidateDCs = ArrayBuffer.empty[CandidateLocation]
    (1 to numberOfLocations).foreach(i=> {
      val numberStrings = iter.next().split(" +|\t")
      val index = numberStrings(0).toInt
      val demand = numberStrings(1).toDouble
      val emergencyCost = numberStrings(2).toDouble
      val fixedCost = numberStrings(3).toDouble
      val lat = numberStrings(4).toDouble
      val lng = - numberStrings(5).toDouble // negative convert to standard longitude 
      demands += new DemandPoint(index, demand, emergencyCost, lat, lng)
      candidateDCs += new CandidateLocation(index, fixedCost, lat, lng)
      
    })
    (demands.toIndexedSeq, candidateDCs.toIndexedSeq)
  }
}