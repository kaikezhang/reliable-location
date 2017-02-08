package kaike.reliable.location.data

import kaike.reliable.location.model.SolverCommon

case class LocationSolution(instance: ProblemInstance, 
                            openDCs: Seq[CandidateLocation], 
                            assignments: Seq[(DemandPoint, CandidateLocation)], 
                            time:Double,
                            solver:SolverCommon,
                            objValue:Double,
                            gap: Double,
                            status:String = "Stop criteria meet");