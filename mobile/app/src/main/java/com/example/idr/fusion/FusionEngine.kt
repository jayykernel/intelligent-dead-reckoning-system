package com.example.idr.fusion

import com.example.idr.calibration.CalibrationEngine
import com.example.idr.nhc_zupt.LeanAngleEKF
import com.example.idr.nhc_zupt.ConstrainedINS
import com.example.idr.outage_prediction.OutagePredictor
import com.example.idr.map_matching.HMMMapMatcher
import com.example.idr.map_matching.RoadNetwork

import org.ejml.simple.SimpleMatrix
import kotlin.math.*

/**
 * Kotlin port of GNSS+INS Fusion Engine.
 */
class FusionEngine(
    val dt: Double = 0.1,
    val enableAi: Boolean = true
) {
    val ekf = ErrorStateEKF(dt = dt)
    val calib = CalibrationEngine()
    val leanEkf = LeanAngleEKF(dt = dt)
    val constrainedIns = ConstrainedINS(dt = dt)
    val aiCorrector = AICorrectionModule()
    val outagePredictor = OutagePredictor()

    // Modules placeholders for TFLite (Phase 11: integrate later)
    var classifier: Any? = null
    var mapMatcher: HMMMapMatcher? = null

    var currentVehicleType = "car"
    var currentLeanAngleRad = 0.0

    // Mode handler state
    var modeCurrentState = "GNSS_AIDED"
    var modeTimeInState = 0.0

    fun step(
        accRaw: DoubleArray,
        gyroRaw: DoubleArray,
        gnssPosEnu: DoubleArray?,
        gnssVelEnu: DoubleArray?,
        isGnssAvailable: Boolean,
        timestamp: Double,
        gnssAccM: Double?,
        gnssSatCount: Int?,
        gnssAvgCn0: Double?
    ): Map<String, Any> {
        // 1. Calibration
        val (accVeh, gyroVeh) = calib.apply(accRaw, gyroRaw)

        // 3. AI Correction
        val (aiSpeed, sigmaAi, qScale) = aiCorrector.processImuSample(accRaw, gyroRaw)

        // 4. EKF Prop
        ekf.predict(accVeh, gyroVeh, dt, qScale)

        // 5. Lean Angle
        val R = ErrorStateEKF.quatToRot(ekf.q)
        val vVehMat = R.transpose().mult(SimpleMatrix(3, 1, false, *ekf.v))
        val vVeh = doubleArrayOf(vVehMat.get(0, 0), vVehMat.get(1, 0), vVehMat.get(2, 0))
        val fwdSpeed = vVeh[1]

        if (currentVehicleType == "two_wheeler") {
            currentLeanAngleRad = leanEkf.update(accVeh[0], accVeh[2], fwdSpeed, gyroVeh[2])
        } else {
            currentLeanAngleRad = 0.0
        }

        // 6. NHC / ZUPT
        if (constrainedIns.isStopped(accVeh, gyroVeh, vVeh)) {
            ekf.updateZupt(0.05)
        } else {
            ekf.updateNhc(currentVehicleType, currentLeanAngleRad, 0.2, 0.2)
        }

        if (aiSpeed != null) {
            ekf.updateAiForwardSpeed(aiSpeed, sigmaAi)
        }

        // 8. GNSS Updates
        val trustScore = if (isGnssAvailable) {
            outagePredictor.update(gnssAvgCn0 ?: 0.0, gnssSatCount ?: 0, gnssAccM ?: 100.0)
        } else {
            0.0
        }

        // Mode handler
        modeTimeInState += dt
        if (modeCurrentState == "GNSS_AIDED" && trustScore < 0.2 && modeTimeInState >= 1.0) {
            modeCurrentState = "PURE_DEAD_RECKONING"
            modeTimeInState = 0.0
        } else if (modeCurrentState == "PURE_DEAD_RECKONING" && trustScore > 0.8 && modeTimeInState >= 1.0) {
            modeCurrentState = "GNSS_AIDED"
            modeTimeInState = 0.0
        }

        var gnssPosPassed = false
        var gnssVelPassed = false
        if (modeCurrentState == "GNSS_AIDED" && isGnssAvailable && gnssPosEnu != null) {
            val dynamicSigmaPos = 5.0 / max(0.1, sqrt(trustScore))
            val (posPassed, _, _) = ekf.updateGnssPosition(gnssPosEnu, dynamicSigmaPos)
            gnssPosPassed = posPassed

            if (gnssVelEnu != null) {
                val dynamicSigmaVel = 0.5 / max(0.2, trustScore)
                val (velPassed, _, _) = ekf.updateGnssVelocity(gnssVelEnu, dynamicSigmaVel)
                gnssVelPassed = velPassed

                val speed2d = hypot(gnssVelEnu[0], gnssVelEnu[1])
                if (speed2d >= 1.0) {
                    val sigmaHeading = if (speed2d >= 1.5) radians(3.0) else radians(3.0 + 7.0 * (1.5 - speed2d) / 0.5)
                    val cogHeading = atan2(gnssVelEnu[0], gnssVelEnu[1])
                    ekf.updateHeading(cogHeading, sigmaHeading, "GNSS_HEADING")
                }
            }
        }

        // Compute heading (yaw) from quaternion
        val headingDeg = when {
            ekf.q.size == 4 -> {
                val q = ekf.q
                val siny_cosp = 2.0 * (q[3] * q[2] + q[0] * q[1])
                val cosy_cosp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
                atan2(siny_cosp, cosy_cosp) * 180.0 / PI
            }
            else -> 0.0
        }

        // Compute overall GNSS pass status: if we have input, we require it to have passed
        val gnssPassed = (gnssPosEnu == null || gnssPosPassed) && (gnssVelEnu == null || gnssVelPassed)

        return mapOf(
            "pos" to ekf.p.copyOf(),
            "vel" to ekf.v.copyOf(),
            "mode" to modeCurrentState,
            "gnss_pos_passed" to gnssPosPassed,
            "gnss_vel_passed" to gnssVelPassed,
            "gnss_passed" to gnssPassed,
            "trust_score" to trustScore,
            "cov_2d" to ekf.getPositionCovariance2d(),
            "heading" to headingDeg
        )
    }

    private fun radians(deg: Double) = deg * Math.PI / 180.0
}
