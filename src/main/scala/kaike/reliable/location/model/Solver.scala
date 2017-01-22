package kaike.reliable.location.model

import kaike.reliable.location.data.LocationSolution


abstract class Solver(val SOLVER_NAME:String) {
  def solve():Option[LocationSolution]
}