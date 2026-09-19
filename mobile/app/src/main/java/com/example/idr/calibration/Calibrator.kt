package com.example.idr.calibration

import org.ejml.simple.SimpleMatrix
import kotlin.math.acos
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Estimates Phone-to-Vehicle frame alignment and IMU biases.
 * Port of engine/calibration/calibrator.py
 */
class CalibrationEngine {
    var gyroBias: DoubleArray = doubleArrayOf(0.0, 0.0, 0.0)
    var accelBias: DoubleArray = doubleArrayOf(0.0, 0.0, 0.0)
    var rPhoneToVeh: SimpleMatrix = SimpleMatrix.identity(3)

    var isCalibrated: Boolean = false
    var alignmentScore: Double = 0.0

    /**
     * Run calibration on a segment of data (e.g., first 120 seconds).
     * @param acc Array of 3D acceleration samples [N][3]
     * @param gyro Array of 3D gyroscope samples [N][3]
     * @param speed Array of speed samples [N]
     * @param dt Sampling period (seconds)
     * @return True if calibration succeeded
     */
    fun calibrateFromSession(
        acc: Array<DoubleArray>,
        gyro: Array<DoubleArray>,
        speed: DoubleArray,
        dt: Double = 0.1
    ): Boolean {
        val n = acc.size
        if (n == 0) return false

        // 1. Isolate stationary periods for Gyro Bias and Gravity Down
        var stationaryIndices = ArrayList<Int>()
        for (i in 0 until n) {
            if (speed[i] < 0.2) {
                stationaryIndices.add(i)
            }
        }

        if (stationaryIndices.size < 20) {
            // Relax threshold
            stationaryIndices.clear()
            for (i in 0 until n) {
                if (speed[i] < 0.5) {
                    stationaryIndices.add(i)
                }
            }
        }

        if (stationaryIndices.size < 10) {
            return false
        }

        // Compute mean stationary gyro (gyro bias)
        val meanGyro = DoubleArray(3)
        val meanAcc = DoubleArray(3)
        for (idx in stationaryIndices) {
            for (j in 0..2) {
                meanGyro[j] += gyro[idx][j]
                meanAcc[j] += acc[idx][j]
            }
        }
        for (j in 0..2) {
            meanGyro[j] /= stationaryIndices.size.toDouble()
            meanAcc[j] /= stationaryIndices.size.toDouble()
        }

        gyroBias = meanGyro
        val gPhone = meanAcc

        val gNorm = norm(gPhone)
        if (gNorm < 1e-6) return false
        val zVPhone = DoubleArray(3) { gPhone[it] / gNorm }

        // 2. Isolate forward acceleration periods for Forward Axis (np.gradient equivalent)
        val ds = DoubleArray(n)
        if (n >= 2) {
            ds[0] = (speed[1] - speed[0]) / dt
            ds[n - 1] = (speed[n - 1] - speed[n - 2]) / dt
            for (i in 1 until n - 1) {
                ds[i] = (speed[i + 1] - speed[i - 1]) / (2.0 * dt)
            }
        }

        var accelIndices = ArrayList<Int>()
        for (i in 0 until n) {
            if (ds[i] > 0.5) {
                accelIndices.add(i)
            }
        }

        if (accelIndices.size < 10) {
            accelIndices.clear()
            for (i in 0 until n) {
                if (ds[i] > 0.2) {
                    accelIndices.add(i)
                }
            }
            if (accelIndices.size < 10) {
                return false
            }
        }

        // Average the forward acceleration in phone frame
        val aFwdPhone = DoubleArray(3)
        for (idx in accelIndices) {
            for (j in 0..2) {
                aFwdPhone[j] += (acc[idx][j] - gPhone[j])
            }
        }
        for (j in 0..2) {
            aFwdPhone[j] /= accelIndices.size.toDouble()
        }

        // Project onto the horizontal plane (orthogonal to Z)
        val dotProduct = dot(aFwdPhone, zVPhone)
        val yVPhone = DoubleArray(3) { aFwdPhone[it] - dotProduct * zVPhone[it] }
        val normY = norm(yVPhone)
        if (normY < 0.05) {
            return false
        }

        for (j in 0..2) {
            yVPhone[j] /= normY
        }

        // 3. Compute X (Right) = cross(y, z)
        val xVPhone = cross(yVPhone, zVPhone)

        // 4. Construct Rotation Matrix: R_veh_to_phone has columns [x, y, z]
        val rVehToPhone = SimpleMatrix(3, 3)
        for (i in 0..2) {
            rVehToPhone.set(i, 0, xVPhone[i])
            rVehToPhone.set(i, 1, yVPhone[i])
            rVehToPhone.set(i, 2, zVPhone[i])
        }

        rPhoneToVeh = rVehToPhone.transpose()
        alignmentScore = min(1.0, accelIndices.size.toDouble() / 50.0)
        isCalibrated = true

        accelBias = doubleArrayOf(0.0, 0.0, gNorm - 9.80665)
        return true
    }

    /**
     * Apply calibration to convert raw phone IMU to vehicle-frame IMU.
     * @return Pair of (accVeh, gyroVeh) as DoubleArray(3)
     */
    fun apply(acc: DoubleArray, gyro: DoubleArray): Pair<DoubleArray, DoubleArray> {
        // Gyro: subtract bias in phone frame, then rotate to vehicle frame
        val gyroCorr = SimpleMatrix(3, 1, true, *doubleArrayOf(
            gyro[0] - gyroBias[0],
            gyro[1] - gyroBias[1],
            gyro[2] - gyroBias[2]
        ))
        val gyroVehMat = rPhoneToVeh.mult(gyroCorr)

        // Acc: rotate raw acc to vehicle frame, then subtract accelBias
        val accMat = SimpleMatrix(3, 1, true, *acc)
        val accVehMat = rPhoneToVeh.mult(accMat)

        val accVeh = doubleArrayOf(
            accVehMat.get(0, 0) - accelBias[0],
            accVehMat.get(1, 0) - accelBias[1],
            accVehMat.get(2, 0) - accelBias[2]
        )
        val gyroVeh = doubleArrayOf(
            gyroVehMat.get(0, 0),
            gyroVehMat.get(1, 0),
            gyroVehMat.get(2, 0)
        )

        return Pair(accVeh, gyroVeh)
    }

    private fun norm(v: DoubleArray): Double = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])

    private fun dot(a: DoubleArray, b: DoubleArray): Double = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    private fun cross(a: DoubleArray, b: DoubleArray): DoubleArray = doubleArrayOf(
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    )
}
