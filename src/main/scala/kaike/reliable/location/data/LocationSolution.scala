package kaike.reliable.location.data

case class LocationSolution(instance: ProblemInstance, 
                            openDCs: Seq[CandidateLocation], 
                            assignments: Seq[(DemandPoint, CandidateLocation)], 
                            time:Double,
                            solver:String,
                            objValue:Double);