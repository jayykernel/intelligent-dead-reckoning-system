package com.example.idr.outage_prediction

import java.util.ArrayDeque
import kotlin.math.abs

/**
 * Predictive Outage Detection (Phase 8).
 * Maintains a short trailing trend of Android raw GNSS measurements.
 * Port of engine/outage_prediction/outage_predictor.py
 */
class OutagePredictor(
    val windowSizeSec: Double = 3.0,
    val dt: Double = 1.0
) {
    private val maxLen = maxOf(3, (windowSizeSec / dt).toInt())

    private val cn0Buffer = ArrayDeque<Double>(maxLen)
    private val satCountBuffer = ArrayDeque<Int>(maxLen)
    private val accBuffer = ArrayDeque<Double>(maxLen)

    var currentTrust: Double = 1.0

    private val strongCn0 = 35.0
    private val weakCn0 = 20.0
    private val goodSatCount = 12
    private val poorSatCount = 4

    private fun addOrTrim(buffer: ArrayDeque<Double>, value: Double) {
        if (buffer.size == maxLen) buffer.removeFirst()
        buffer.addLast(value)
    }

    private fun addOrTrimInt(buffer: ArrayDeque<Int>, value: Int) {
        if (buffer.size == maxLen) buffer.removeFirst()
        buffer.addLast(value)
    }

    private fun calculateTrend(buffer: List<Double>, isInverted: Boolean = false): Double {
        val n = buffer.size
        if (n < 2) return 0.0

        var sumX = 0.0
        var sumY = 0.0
        var sumXY = 0.0
        var sumX2 = 0.0

        for (i in 0 until n) {
            val x = i.toDouble()
            val y = buffer[i]
            sumX += x
            sumY += y
            sumXY += x * y
            sumX2 += x * x
        }

        val denom = n * sumX2 - sumX * sumX
        if (denom == 0.0) return 0.0

        val slope = (n * sumXY - sumX * sumY) / denom

        return if (isInverted) -slope else slope
    }

    fun update(
        avgCn0: Double? = null,
        satCount: Int? = null,
        accuracyM: Double? = null
    ): Double {
        if (avgCn0 != null) addOrTrim(cn0Buffer, avgCn0)
        if (satCount != null) addOrTrimInt(satCountBuffer, satCount)
        if (accuracyM != null) addOrTrim(accBuffer, accuracyM)

        if (cn0Buffer.size < 2 && satCountBuffer.size < 2) {
            currentTrust = 1.0
            if (satCount != null && satCount <= poorSatCount) {
                currentTrust = 0.1
            }
            return currentTrust
        }

        val trustLevels = mutableListOf<Double>()

        if (cn0Buffer.isNotEmpty()) {
            val cn0 = cn0Buffer.last
            var tCn0 = (cn0 - weakCn0) / (strongCn0 - weakCn0)
            tCn0 = tCn0.coerceIn(0.0, 1.0)
            trustLevels.add(tCn0)
        }

        if (satCountBuffer.isNotEmpty()) {
            val sats = satCountBuffer.last
            val denom = maxOf(1, goodSatCount - poorSatCount).toDouble()
            var tSats = (sats - poorSatCount) / denom
            tSats = tSats.coerceIn(0.0, 1.0)
            trustLevels.add(tSats)
        }

        val baseTrust = if (trustLevels.isNotEmpty()) trustLevels.average() else 1.0

        var trendPenalty = 0.0

        if (satCountBuffer.size >= 2) {
            val satTrend = calculateTrend(satCountBuffer.map { it.toDouble() })
            if (satTrend < -0.99999999) {
                val penalty = (abs(satTrend) * 0.1).coerceIn(0.0, 0.3)
                trendPenalty += penalty
            }
        }

        if (cn0Buffer.size >= 2) {
            val cn0Trend = calculateTrend(cn0Buffer.toList())
            if (cn0Trend < -1.99999999) {
                val penalty = (abs(cn0Trend) * 0.05).coerceIn(0.0, 0.2)
                trendPenalty += penalty
            }
        }

        currentTrust = (baseTrust - trendPenalty).coerceIn(0.0, 1.0)
        return currentTrust
    }

    fun reset() {
        cn0Buffer.clear()
        satCountBuffer.clear()
        accBuffer.clear()
        currentTrust = 1.0
    }
}
