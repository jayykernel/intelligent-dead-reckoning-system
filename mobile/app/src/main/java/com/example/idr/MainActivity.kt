package com.example.idr

import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.util.Log // Add this
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import com.example.idr.fusion.FusionEngine
import com.example.idr.ui.ConfidenceEllipseView

/**
 * Main navigation activity with live Confidence Ellipse UI rendering (N4)
 * and real-time Native Kotlin Fusion Engine integration (Phase 11).
 */
class MainActivity : AppCompatActivity() {

    private val TAG = "IDR_Main"

    private lateinit var ellipseView: ConfidenceEllipseView
    private lateinit var tvFilterStatus: TextView
    private lateinit var tvCovarianceInfo: TextView

    private val fusionEngine = FusionEngine(dt = 0.1, enableAi = true)
    private val mainHandler = Handler(Looper.getMainLooper())
    private var isPlaying = false
    private var simTime = 0.0
    private var simEpoch = 0

    private val playbackRunnable = object : Runnable {
        override fun run() {
            if (!isPlaying) return

            simEpoch++
            simTime += 0.1

            if (simEpoch % 10 == 0) { // Log every 1 second (10Hz)
                Log.d(TAG, "Playback stepping: epoch=$simEpoch, time=$simTime")
            }

            // Multi-phase test drive loop (26.0s total cycle):
            // 0..60 (0-6s): Good GNSS (Open sky, GREEN)
            // 60..110 (6-11s): Continuous signal degradation approaching tunnel (AMBER)
            // 110..190 (11-19s): Total GNSS outage inside tunnel (RED)
            // 190..230 (19-23s): GNSS reacquisition / NIS rejection exiting tunnel (PURPLE & AMBER)
            // 230..260 (23-26s): Re-converged steady-state navigation (GREEN)
            val cycleEpoch = simEpoch % 260

            val (gnssPos, gnssVel, isGnssAvail, cn0, sats, accM) = when {
                cycleEpoch < 60 -> {
                    val pos = doubleArrayOf(10.0 * simTime, 0.0, 0.0)
                    val vel = doubleArrayOf(10.0, 0.0, 0.0)
                    SimData(pos, vel, true, 38.0, 16, 2.0)
                }
                cycleEpoch < 110 -> {
                    // Continuous linear degradation ramp over 5.0s (50 epochs)
                    val progress = (cycleEpoch - 60) / 50.0
                    val pos = doubleArrayOf(10.0 * simTime, 0.0, 0.0)
                    val vel = doubleArrayOf(10.0, 0.0, 0.0)
                    val currCn0 = 38.0 - progress * (38.0 - 15.0)
                    val currSats = (16.0 - progress * (16.0 - 3.0)).toInt()
                    val currAccM = 2.0 + progress * (25.0 - 2.0)
                    SimData(pos, vel, true, currCn0, currSats, currAccM)
                }
                cycleEpoch < 190 -> {
                    // Total GNSS blackout in tunnel
                    SimData(null, null, false, 0.0, 0, 99.0)
                }
                cycleEpoch < 230 -> {
                    // GNSS signal reappears upon tunnel exit, recovering over 4.0s (40 epochs)
                    val reacqProgress = (cycleEpoch - 190) / 40.0
                    val pos = doubleArrayOf(10.0 * simTime, 0.0, 0.0)
                    val vel = doubleArrayOf(10.0, 0.0, 0.0)
                    val currCn0 = 20.0 + reacqProgress * (38.0 - 20.0)
                    val currSats = (4.0 + reacqProgress * (16.0 - 4.0)).toInt()
                    val currAccM = 22.0 - reacqProgress * (22.0 - 2.0)
                    SimData(pos, vel, true, currCn0, currSats, currAccM)
                }
                else -> {
                    val pos = doubleArrayOf(10.0 * simTime, 0.0, 0.0)
                    val vel = doubleArrayOf(10.0, 0.0, 0.0)
                    SimData(pos, vel, true, 39.0, 18, 1.8)
                }
            }

            val result = fusionEngine.step(
                accRaw = doubleArrayOf(0.0, 0.0, 9.81),
                gyroRaw = doubleArrayOf(0.0, 0.0, 0.0),
                gnssPosEnu = gnssPos,
                gnssVelEnu = gnssVel,
                isGnssAvailable = isGnssAvail,
                timestamp = simTime,
                gnssAccM = accM,
                gnssSatCount = sats,
                gnssAvgCn0 = cn0
            )

            val cov2d = result["cov_2d"] as DoubleArray
            val mode = result["mode"] as String
            val trust = (result["trust_score"] as Double).toFloat()
            val heading = (result["heading"] as Double).toFloat()
            val gnssPassed = result["gnss_passed"] as Boolean

            // Amber state logging: log when trust is in the degrading band (0.2 - 0.8)
            if (trust >= 0.2f && trust <= 0.8f) {
                Log.d(TAG, ">>> AMBER STATE ACTIVE: Trust=$trust, Mode=$mode <<<")
            }

            updateNavigationState(
                pEE = cov2d[0],
                pNN = cov2d[1],
                pEN = cov2d[2],
                mode = mode,
                trust = trust,
                heading = heading,
                nisPassed = gnssPassed
            )

            mainHandler.postDelayed(this, 100) // 10 Hz update rate
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        ellipseView = findViewById(R.id.confidenceEllipseView)
        tvFilterStatus = findViewById(R.id.tvFilterStatus)
        tvCovarianceInfo = findViewById(R.id.tvCovarianceInfo)

        // Initialize fusion engine state
        fusionEngine.ekf.p = doubleArrayOf(0.0, 0.0, 0.0)
        fusionEngine.ekf.v = doubleArrayOf(10.0, 0.0, 0.0)

        // Start 10Hz live navigation pipeline playback
        startPlayback()
    }

    override fun onResume() {
        super.onResume()
        if (!isPlaying) {
            startPlayback()
        }
    }

    override fun onPause() {
        super.onPause()
        stopPlayback()
    }

    fun startPlayback() {
        isPlaying = true
        mainHandler.post(playbackRunnable)
    }

    fun stopPlayback() {
        isPlaying = false
        mainHandler.removeCallbacks(playbackRunnable)
    }

    /**
     * Updates the UI when a new state estimate is produced by the fusion pipeline.
     */
    fun updateNavigationState(
        pEE: Double,
        pNN: Double,
        pEN: Double,
        mode: String,
        trust: Float,
        heading: Float,
        nisPassed: Boolean
    ) {
        ellipseView.updateState(
            pEE = pEE,
            pNN = pNN,
            pEN = pEN,
            currentMode = mode,
            trust = trust,
            heading = heading,
            nisPassed = nisPassed
        )

        val trace = pEE + pNN
        val nisText = if (nisPassed) "PASS" else "FAIL"
        tvFilterStatus.text = "Mode: $mode | Trust: ${(trust * 100).toInt()}% | NIS: $nisText"
        tvCovarianceInfo.text = String.format("Pos Var Trace: %.2f m² | P_EE=%.2f, P_NN=%.2f", trace, pEE, pNN)
    }

    private data class SimData(
        val pos: DoubleArray?,
        val vel: DoubleArray?,
        val isAvail: Boolean,
        val cn0: Double,
        val sats: Int,
        val accM: Double
    )
}
