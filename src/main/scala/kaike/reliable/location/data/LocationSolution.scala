package kaike.reliable.location.data

import kaike.reliable.location.model.Solver

case class LocationSolution(instance: ProblemInstance, 
                            openDCs: Seq[CandidateLocation], 
                            assignments: Seq[(DemandPoint, CandidateLocation)], 
                            time:Double,
                            solver:Solver,
                            objValue:Double,
                            gap: Double);