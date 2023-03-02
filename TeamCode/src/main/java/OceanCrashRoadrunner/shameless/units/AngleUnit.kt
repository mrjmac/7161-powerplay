package OceanCrashRoadrunner.shameless.units

import kotlin.math.PI

enum class AngleUnit(private val degreesConversionFactor: Double) {
    DEGREES    (degreesConversionFactor = 1.0);

    fun toDeg(x: Number) = x.toDouble() * degreesConversionFactor

    fun toRad(x: Number) = toDeg(x) * PI / 180

    fun to(unit: AngleUnit, x: Number) = unit.toDeg(x) / degreesConversionFactor
}