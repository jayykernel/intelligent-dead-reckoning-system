package com.example.idr.fusion

import org.json.JSONArray
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import org.ejml.simple.SimpleMatrix
import java.io.File

class ErrorStateEKFTest {

    @Test
    fun testErrorStateEKFNumericalParity() {
        val inPath = "../../engine/fusion/tests/vectors/ekf_out.json"
        val file = File(inPath)

        assertTrue("Test vector file not found: ${file.absolutePath}", file.exists())

        val outJson = JSONArray(file.readText())

        val ekf = ErrorStateEKF(dt = 0.1)

        // Set initial state to match test vectors
        val q0 = doubleArrayOf(0.9238795, 0.0, 0.0, 0.3826834) // approx 45 deg yaw
        ekf.setInitialState(
            p0 = doubleArrayOf(10.0, 20.0, 0.0),
            v0 = doubleArrayOf(5.0, 0.0, 0.0),
            q0 = q0,
            bA0 = doubleArrayOf(0.1, -0.05, 0.02),
            bG0 = doubleArrayOf(0.01, 0.02, -0.01)
        )

        for (stepIdx in 0 until outJson.length()) {
            val stepObj = outJson.getJSONObject(stepIdx)
            val type = stepObj.getString("type")

            when (type) {
                "predict" -> {
                    val accRaw = doubleArrayOf(0.1, 9.8, 0.2)
                    val gyroRaw = doubleArrayOf(0.01, 0.02, 0.1)
                    ekf.predict(accRaw, gyroRaw)
                }
                "gnss_pos" -> {
                    ekf.updateGnssPosition(doubleArrayOf(10.5, 20.0, 0.0), sigmaPos = 2.0)
                }
                "gnss_vel" -> {
                    ekf.updateGnssVelocity(doubleArrayOf(4.9, -0.1, 0.0))
                }
                "heading" -> {
                    ekf.updateHeading(headingRad = 0.785398) // ~45 deg
                }
                "ai_speed" -> {
                    ekf.updateAiForwardSpeed(speedFwd = 12.0)
                }
                "zupt" -> {
                    ekf.updateZupt()
                }
                "nhc" -> {
                    ekf.updateNhc(vehicleType = "two_wheeler", leanAngleRad = 0.1)
                }
                else -> throw IllegalArgumentException("Unknown step type: $type")
            }

            // Compare state
            val expP = stepObj.getJSONArray("p")
            val expV = stepObj.getJSONArray("v")
            val expQ = stepObj.getJSONArray("q")
            val expBA = stepObj.getJSONArray("ba")
            val expBG = stepObj.getJSONArray("bg")
            val expPMatJSON = stepObj.getJSONArray("P")

            for (i in 0..2) {
                assertEquals("p[$i] mismatch at step $stepIdx (type=$type)", expP.getDouble(i), ekf.p[i], 1e-5)
                assertEquals("v[$i] mismatch at step $stepIdx (type=$type)", expV.getDouble(i), ekf.v[i], 1e-5)
                assertEquals("q[$i] mismatch at step $stepIdx (type=$type)", expQ.getDouble(i), ekf.q[i], 1e-5)
                assertEquals("bA[$i] mismatch at step $stepIdx (type=$type)", expBA.getDouble(i), ekf.bA[i], 1e-5)
                assertEquals("bG[$i] mismatch at step $stepIdx (type=$type)", expBG.getDouble(i), ekf.bG[i], 1e-5)
            }

            val expPMat = SimpleMatrix(15, 15)
            for (r in 0..14) {
                val row = expPMatJSON.getJSONArray(r)
                for (c in 0..14) {
                    expPMat.set(r, c, row.getDouble(c))
                }
            }

            for (r in 0..14) {
                for (c in 0..14) {
                    val expVal = expPMat.get(r, c)
                    val actVal = ekf.P.get(r, c)
                    assertEquals("P[$r][$c] mismatch at step $stepIdx (type=$type)", expVal, actVal, 1e-5)
                }
            }
        }
    }
}
