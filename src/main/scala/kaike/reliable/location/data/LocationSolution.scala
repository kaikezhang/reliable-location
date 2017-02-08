package kaike.reliable.location.data

import kaike.reliable.location.model.LocationProblemSolver

case class LocationSolution(instance: ProblemInstance, 
                            openDCs: Seq[CandidateLocation], 
                            assignments: Seq[(DemandPoint, CandidateLocation)], 
                            time:Double,
                            solver:LocationProblemSolver,
                            objValue:Double,
                            gap: Double,
                            status:String = "Stop criteria meet");