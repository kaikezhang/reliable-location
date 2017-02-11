package kaike.reliable.location

package object main {
  def trim(s:String):String = {
    if(s.size > 1E6){
      s.take(3E5.toInt) + (1 to 10).map(x => "\n").mkString + "content ommited" + (1 to 10).map(x => "\n").mkString + s.takeRight(3E5.toInt)
    } else {
      s
    }
  }
}