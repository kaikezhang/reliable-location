package kaike.reliable.location.data

import scala.collection.immutable.TreeSet

import org.json4s.JsonAST.JArray
import org.json4s.JsonAST.JDouble
import org.json4s.JsonAST.JInt

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

case class RobustReliableLocationParameter(alpha:Double = 1, theta:Int = 400){
  override def toString() = {
    s"Alpha = ${"%.2f".format(alpha)}\nTheta = ${theta}"
  }
}
case class StochasticReliableLocationParameter(alpha:Double = 1) {
  override def toString() = {
    s"Alpha = ${"%.2f".format(alpha)}"
  }  
}
case class CrossMomentParameter(alpha: Double = 1, theta:Int = 400, matrixType:Int = 1) {
  override def toString() = {
    s"Alpha = ${"%.2f".format(alpha)}\nTheta = ${theta}\nMatrixType = ${matrixType}"
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
                                            parameter: StochasticReliableLocationParameter = StochasticReliableLocationParameter()) 
                                                          extends ReliableProblemInstance(demandPoints, candidateLocations, "Stochastic RUFLP"){

  val alpha = parameter.alpha
  val theta = 400 //
  
  val newOrleans = Coordinate(30.07, -89.93)
  val failRate = candidateLocationIndexes.map { i => Math.min(1, 0.01 + 0.1 * alpha * Math.exp(-(GeoComputer.distance(candidateLocations(i), newOrleans) / theta))) }

}

case class RobustReliableLocationProblemInstance( override val demandPoints: IndexedSeq[DemandPoint],  
                                          override val candidateLocations: IndexedSeq[CandidateLocation], 
                                          parameter: RobustReliableLocationParameter = RobustReliableLocationParameter())
                                                          extends ReliableProblemInstance(demandPoints, candidateLocations, "Robust RUFLP Marginal"){
  
  val alpha = parameter.alpha
  val theta = parameter.theta
  
  val newOrleans = Coordinate(30.07, -89.93)
  val failRate = candidateLocationIndexes.map { i => Math.min(1,  alpha * Math.exp(-(GeoComputer.distance(candidateLocations(i), newOrleans) / theta))) }

}

class Scenario(val failures:TreeSet[Int], var prob:Double){
  def this(failuresSeq: Seq[Int], prob:Double) = {
    this(failuresSeq.to[TreeSet], prob)
  }
}
  
case class CrossMomentProblemInstance(override val demandPoints: IndexedSeq[DemandPoint],
                                       override val candidateLocations: IndexedSeq[CandidateLocation], 
                                       parameter: CrossMomentParameter = CrossMomentParameter())
                                                          extends ProblemInstance(demandPoints, candidateLocations, "Robust RUFLP Crossmoment") {
  val alpha = parameter.alpha
  val theta = parameter.theta
  val newOrleans = Coordinate(30.07, -89.93)
  
  println("""
failRate = candidateLocationIndexes.map { i => Math.min(1, 0.01 + 0.1  * alpha  * Math.exp(-(GeoComputer.distance(candidateLocations(i), newOrleans) / theta))) }
  """)
  val failRate = candidateLocationIndexes.map { i => Math.min(1, 0.01 + 0.1 * alpha * Math.exp(-(GeoComputer.distance(candidateLocations(i), newOrleans) / theta))) }
  
  private val candidateDistance = Array.tabulate(candidateLocations.length, candidateLocations.length)((i,j) => {
    GeoComputer.distance(candidateLocations(i), candidateLocations(j))
  })

  private val nearestLoc = candidateLocationIndexes.map(i => candidateLocationIndexes.filter(j => j != i).minBy(j => candidateDistance(i)(j)) )
  
  def generateCrossMomentMatrixPattern1() = {
  println("""
crossMomentMatrix === (i, j) => {
    if(i == j)
      failRate(i)
    else 
      - 1
} 
  """)    
    Array.tabulate(candidateLocations.size, candidateLocations.size)((i, j) => {
      if(i == j)
        failRate(i)
      else 
        - 1
    })    
  }
  
  def generateCrossMomentMatrixPattern2() = {
  println("""
crossMomentMatrix === (i, j) => {
    if(i == j)
      failRate(i)
    else if(j == nearestLoc(i) || i == nearestLoc(j)){
      failRate(i) * failRate(j) * alpha
    } else
      -1
} 
  """)    
    Array.tabulate(candidateLocations.size, candidateLocations.size)((i, j) => {
      if(i == j)
        failRate(i)
      else if(j == nearestLoc(i) || i == nearestLoc(j)){
        failRate(i) * failRate(j) * alpha
      } else
        -1
    })    
  }  

  def random(a:Double, b:Double) = {
    a + (b-a) * Math.random()
  }
  def generateCrossMomentMatrixPattern3() = {
  println("""
crossMomentMatrix === (i, j) => {
    if(i == j)
      failRate(i)
    else
      failRate(i) * failRate(j) * random(0.5, 1.5)
}
  """)    
    Array.tabulate(candidateLocations.size, candidateLocations.size)((i, j) => {
      if(i == j)
        failRate(i)
      else
        failRate(i) * failRate(j) * random(0.5, 1.5)
    })    
  } 
  
  
  val crossMomentMatrix:Array[Array[Double]] = parameter.matrixType match {
    case 1 => generateCrossMomentMatrixPattern1()
    case 2 => generateCrossMomentMatrixPattern2()
    case 3 => generateCrossMomentMatrixPattern3()
    case _ => {
      throw new IllegalArgumentException("Pattern not supported")
    }
  }
  
  println("Generated cross moment matrix:")
  crossMomentMatrix.foreach { x => println(x.map { x => x.toString.padTo(25, " ").mkString }.mkString("[", ", ", "]")) }
  println()
  
  val nbRealizations = candidateLocations.size
  
  def generateRandomScenario(): Scenario = {
    val failures = candidateLocationIndexes.filter { j => Math.random() > 0.5 }
    new Scenario(failures, 0)
  }
  
  def generateSingletonScenario(i: Int): Scenario = {
    val failures = candidateLocationIndexes.filter { j => j == i }
    new Scenario(failures, 0)    
  }
  
  val realizations = (0 until nbRealizations).map(i => 
    generateSingletonScenario(i)
//    generateRandomScenario()
  )
  
}


