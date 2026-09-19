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

            // Multi-phase test drive loop:
            // 0..60 (0-6s): Good GNSS (Open sky)
            // 60..90 (6-9s): Signal degradation (Approaching tunnel)
            // 90..180 (9-18s): Total GNSS outage (Inside tunnel)
            // 180..240 (18-24s): GNSS reacquisition (Exiting tunnel)
            val cycleEpoch = simEpoch % 240

            val (gnssPos, gnssVel, isGnssAvail, cn0, sats, accM) = when {
                cycleEpoch < 60 -> {
                    val pos = doubleArrayOf(10.0 * simTime, 0.0, 0.0)
                    val vel = doubleArrayOf(10.0, 0.0, 0.0)
                    SimData(pos, vel, true, 38.0, 16, 2.0)
                }
                cycleEpoch < 90 -> {
                    val pos = doubleArrayOf(10.0 * simTime, 0.0, 0.0)
                    val vel = doubleArrayOf(10.0, 0.0, 0.0)
                    SimData(pos, vel, true, 16.0, 4, 22.0)
                }
                cycleEpoch < 180 -> {
                    SimData(null, null, false, 0.0, 0, 99.0)
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
