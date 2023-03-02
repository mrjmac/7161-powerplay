package OceanCrashRoadrunner.shameless.util

import OceanCrashRoadrunner.shameless.units.AngleUnit
import OceanCrashRoadrunner.shameless.units.DistanceUnit
import OceanCrashRoadrunner.shameless.units.GlobalUnits
import kotlin.math.PI
import kotlin.math.abs


@JvmOverloads
fun Number.toIn(from: DistanceUnit = GlobalUnits.distance): Double = from.toIn(this)

@JvmOverloads
fun Number.toRad(from: AngleUnit = GlobalUnits.angle): Double = from.toDeg(this) * PI / 180