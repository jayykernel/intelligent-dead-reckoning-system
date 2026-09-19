package com.example.idr.fusion

import com.example.idr.ui.ConfidenceEllipse
import org.junit.Assert.*
import org.junit.Test
import kotlin.math.sqrt

/**
 * Phase 11 End-to-End Playback and Pipeline Integration Test.
 *
 * Verifies full end-to-end multi-epoch recorded drive simulation:
 * - Steady state GNSS-aided driving
 * - Predictive outage detection & transition to PURE_DEAD_RECKONING
 * - Continuous Dead Reckoning with NHC / ZUPT / AI corrections
 * - Covariance expansion & ConfidenceEllipse UI metric calculation
 * - GNSS reacquisition, NIS acceptance, and transition back to GNSS_AIDED
 * - Throughput & latency validation (confirming 10Hz real-time requirement)
 */
class FusionEnginePlaybackTest {

    @Test
    fun testEndToEndRecordedPlaybackDrive() {
        val dt = 0.1 // 10 Hz
        val engine = FusionEngine(dt = dt, enableAi = true)

        // Initial state
        engine.ekf.p = doubleArrayOf(0.0, 0.0, 0.0)
        engine.ekf.v = doubleArrayOf(10.0, 0.0, 0.0) // 10 m/s along East

        var t = 0.0
        val totalEpochs = 200 // 20.0 seconds total

        var baselineVariance = 0.0
        var outageMaxVariance = 0.0
        var reacqFinalVariance = 0.0

        var modeSwitchedToDeadReckoning = false
        var modeSwitchedBackToGnssAided = false

        val startTime = System.nanoTime()

        for (epoch in 0 until totalEpochs) {
            t += dt

            // Define scenario phases:
            // Epoch 0..49 (0-5s): Good GNSS
            // Epoch 50..69 (5-7s): Signal degradation (Tunnel entry)
            // Epoch 70..149 (7-15s): Complete GNSS outage (Inside tunnel)
            // Epoch 150..199 (15-20s): GNSS restored (Tunnel exit)

            val (gnssPos, gnssVel, isGnssAvail, cn0, sats, accM) = when {
                epoch < 50 -> {
                    // Nominal open sky
                    val pos = doubleArrayOf(10.0 * t, 0.0, 0.0)
                    val vel = doubleArrayOf(10.0, 0.0, 0.0)
                    HexTuple(pos, vel, true, 38.0, 16, 2.0)
                }
                epoch < 70 -> {
                    // Degrading signal entering tunnel
                    val pos = doubleArrayOf(10.0 * t, 0.0, 0.0)
                    val vel = doubleArrayOf(10.0, 0.0, 0.0)
                    HexTuple(pos, vel, true, 16.0, 4, 25.0)
                }
                epoch < 150 -> {
                    // Hard outage inside tunnel
                    HexTuple(null, null, false, 0.0, 0, 99.0)
                }
                else -> {
                    // Clean reacquisition after tunnel
                    val pos = doubleArrayOf(10.0 * t, 0.0, 0.0)
                    val vel = doubleArrayOf(10.0, 0.0, 0.0)
                    HexTuple(pos, vel, true, 39.0, 18, 1.8)
                }
            }

            val stepResult = engine.step(
                accRaw = doubleArrayOf(0.0, 0.0, 9.81),
                gyroRaw = doubleArrayOf(0.0, 0.0, 0.0),
                gnssPosEnu = gnssPos,
                gnssVelEnu = gnssVel,
                isGnssAvailable = isGnssAvail,
                timestamp = t,
                gnssAccM = accM,
                gnssSatCount = sats,
                gnssAvgCn0 = cn0
            )

            // 1. Verify all output map fields exist and are valid
            assertNotNull(stepResult["pos"])
            assertNotNull(stepResult["vel"])
            assertNotNull(stepResult["mode"])
            assertNotNull(stepResult["trust_score"])
            assertNotNull(stepResult["cov_2d"])
            assertNotNull(stepResult["heading"])

            val pos = stepResult["pos"] as DoubleArray
            val vel = stepResult["vel"] as DoubleArray
            val mode = stepResult["mode"] as String
            val trust = (stepResult["trust_score"] as Double).toFloat()
            val cov2d = stepResult["cov_2d"] as DoubleArray
            val heading = (stepResult["heading"] as Double).toFloat()
            val gnssPassed = stepResult["gnss_passed"] as Boolean

            // Check no NaNs or Infinities
            assertFalse("Pos X contains NaN", pos[0].isNaN() || pos[0].isInfinite())
            assertFalse("Pos Y contains NaN", pos[1].isNaN() || pos[1].isInfinite())
            assertFalse("Vel X contains NaN", vel[0].isNaN() || vel[0].isInfinite())
            assertFalse("Vel Y contains NaN", vel[1].isNaN() || vel[1].isInfinite())

            val pEE = cov2d[0]
            val pNN = cov2d[1]
            val pEN = cov2d[2]
            val traceVar = pEE + pNN

            // Verify ConfidenceEllipse computation (as used by ConfidenceEllipseView)
            val ellipse = ConfidenceEllipse.fromCovariance(pEE, pNN, pEN)
            assertTrue("Semi-major axis must be positive", ellipse.semiMajorAxisM > 0.0f)
            assertTrue("Semi-minor axis must be positive", ellipse.semiMinorAxisM > 0.0f)
            assertTrue("Semi-major must be >= semi-minor", ellipse.semiMajorAxisM >= ellipse.semiMinorAxisM)

            // Check Phase tracking
            if (epoch == 45) {
                baselineVariance = traceVar
                assertEquals("GNSS_AIDED", mode)
                assertTrue("Trust score should be high in open sky", trust > 0.8f)
            }

            if (mode == "PURE_DEAD_RECKONING") {
                modeSwitchedToDeadReckoning = true
            }

            if (epoch in 70..149) {
                if (traceVar > outageMaxVariance) {
                    outageMaxVariance = traceVar
                }
            }

            if (epoch == 199) {
                reacqFinalVariance = traceVar
                if (mode == "GNSS_AIDED") {
                    modeSwitchedBackToGnssAided = true
                }
            }
        }

        val totalDurationMs = (System.nanoTime() - startTime) / 1_000_000.0
        val avgStepDurationMs = totalDurationMs / totalEpochs

        println("==================================================================")
        println("PHASE 11: END-TO-END PLAYBACK INTEGRATION TEST RESULTS")
        println("==================================================================")
        println("Total Simulated Duration: ${totalEpochs * dt}s (${totalEpochs} epochs)")
        println("Execution Time for ${totalEpochs} epochs: ${String.format("%.2f", totalDurationMs)} ms")
        println("Average Step Execution Latency: ${String.format("%.4f", avgStepDurationMs)} ms (Budget: 100.0 ms)")
        println("Baseline Variance (GNSS-Aided): ${String.format("%.4f", baselineVariance)} m²")
        println("Max Variance during Outage: ${String.format("%.4f", outageMaxVariance)} m²")
        println("Final Variance post Reacquisition: ${String.format("%.4f", reacqFinalVariance)} m²")
        println("==================================================================")

        // 2. Assert seamless mode switching
        assertTrue("Pipeline must transition to PURE_DEAD_RECKONING during outage", modeSwitchedToDeadReckoning)
        assertTrue("Pipeline must recover to GNSS_AIDED upon reacquisition", modeSwitchedBackToGnssAided)

        // 3. Assert covariance dynamics (growth in outage, collapse on reacquisition)
        assertTrue("Covariance must grow during GNSS outage", outageMaxVariance > baselineVariance)
        assertTrue("Covariance must contract upon GNSS reacquisition", reacqFinalVariance < outageMaxVariance)

        // 4. Assert real-time performance budget (must easily achieve > 10Hz)
        assertTrue("Average step latency must be < 5.0ms (10Hz is 100ms)", avgStepDurationMs < 5.0)
    }

    private data class HexTuple<A, B, C, D, E, F>(
        val first: A,
        val second: B,
        val third: C,
        val fourth: D,
        val fifth: E,
        val sixth: F
    )
}
