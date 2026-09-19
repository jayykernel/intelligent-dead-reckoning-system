package com.example.idr.fusion

import org.ejml.simple.SimpleMatrix
import kotlin.math.min
import kotlin.math.sqrt

/**
 * AI Correction Module (N7 - MEMS / Mobile Path).
 * Port of engine/fusion/ai_corrector.py
 */
class AICorrectionModule(
    val windowSize: Int = 10,
    val baseSigmaAi: Double = 1.0
) {
    private val imuBuffer = ArrayDeque<DoubleArray>(windowSize + 1)

    fun processImuSample(
        accRaw: DoubleArray,
        gyroRaw: DoubleArray // Note: gyro not used for vibration in AI logic
    ): Triple<Double?, Double, Double> {
        val sample = accRaw + gyroRaw // (6,)
        imuBuffer.addLast(sample)
        if (imuBuffer.size > windowSize) {
            imuBuffer.removeFirst()
        }

        if (imuBuffer.size < windowSize) {
            return Triple(null, baseSigmaAi, 1.0)
        }

        // 1. Compute vibration energy (std deviation of acc)
        val accWindow = mutableListOf<DoubleArray>()
        for (item in imuBuffer) {
            accWindow.add(doubleArrayOf(item[0], item[1], item[2]))
        }

        val stdAcc = calculateStd(accWindow)
        val totalVib = norm3(stdAcc)

        // Dynamic process noise scaling
        val qScale = 1.0 + min(3.0, totalVib / 1.5)

        // Measurement uncertainty scaling
        val sigmaAi = baseSigmaAi * (1.0 + totalVib / 2.0)

        // 2. TFLite model not implemented yet, returning None for speed
        val predSpeed: Double? = null

        return Triple(predSpeed, sigmaAi, qScale)
    }

    private fun calculateStd(data: List<DoubleArray>): DoubleArray {
        val n = data.size
        if (n < 2) return doubleArrayOf(0.0, 0.0, 0.0)

        val means = doubleArrayOf(0.0, 0.0, 0.0)
        for (row in data) {
            for (i in 0..2) means[i] += row[i]
        }
        for (i in 0..2) means[i] /= n.toDouble()

        val stds = doubleArrayOf(0.0, 0.0, 0.0)
        for (row in data) {
            for (i in 0..2) {
                stds[i] += (row[i] - means[i]) * (row[i] - means[i])
            }
        }
        for (i in 0..2) stds[i] = sqrt(stds[i] / n.toDouble())
        return stds
    }

    private fun norm3(v: DoubleArray): Double = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
}
