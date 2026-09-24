package com.example.idr.nhc_zupt

import org.ejml.simple.SimpleMatrix
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Constrained INS with NHC/ZUPT and Lean-Compensated NHC (N1)
 * Port of engine/nhc_zupt/constrained_ins.py
 */
class ConstrainedINS(
    val dt: Double = 0.1,
    val zuptSpeedThreshold: Double = 1.5,
    val zuptAccThreshold: Double = 1.5,
    val zuptGyroThreshold: Double = 0.3,
    val g: Double = 9.80665
) {

    fun isStopped(accVeh: DoubleArray, gyroVeh: DoubleArray, vVeh: DoubleArray? = null): Boolean {
        val accMag = norm(accVeh)
        val gyroMag = norm(gyroVeh)

        val imuQuiet = (abs(accMag - g) < zuptAccThreshold) && (gyroMag < zuptGyroThreshold)

        if (vVeh != null) {
            val vMag = norm(vVeh)
            return imuQuiet && (vMag < zuptSpeedThreshold)
        }

        return imuQuiet
    }

    fun constrain(
        accVeh: DoubleArray,
        gyroVeh: DoubleArray,
        vVeh: DoubleArray,
        vehicleType: String,
        leanAngleRad: Double = 0.0
    ): DoubleArray {
        if (isStopped(accVeh, gyroVeh, vVeh)) {
            return doubleArrayOf(0.0, 0.0, 0.0)
        }

        return when (vehicleType) {
            "car" -> {
                // Standard NHC (X Right, Y Forward, Z Up)
                // Lateral (X)=0, Vertical (Z)=0
                doubleArrayOf(0.0, vVeh[1], 0.0)
            }
            "two_wheeler" -> {
                // Lean-compensated NHC (N1)
                // Forward is Y-axis. Roll/lean rotation is around Y-axis by lean_angle.
                val phi = leanAngleRad
                val c = cos(phi)
                val s = sin(phi)
                val rY = SimpleMatrix(3, 3, true, *doubleArrayOf(
                    c, 0.0, s,
                    0.0, 1.0, 0.0,
                    -s, 0.0, c
                ))

                val vMat = SimpleMatrix(3, 1, true, *vVeh)
                val vRoad = rY.mult(vMat)

                // Apply NHC in road frame: lateral (x_road) = 0 and vertical (z_road) = 0
                val vRoadConstrained = SimpleMatrix(3, 1, true, *doubleArrayOf(
                    0.0,
                    vRoad.get(1, 0),
                    0.0
                ))

                // Transform back to vehicle frame
                val vConstrained = rY.transpose().mult(vRoadConstrained)
                doubleArrayOf(
                    vConstrained.get(0, 0),
                    vConstrained.get(1, 0),
                    vConstrained.get(2, 0)
                )
            }
            else -> throw IllegalArgumentException("Unknown vehicle type: $vehicleType")
        }
    }

    private fun norm(v: DoubleArray): Double = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
}
