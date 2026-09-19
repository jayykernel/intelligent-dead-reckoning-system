package com.example.idr.calibration

import org.json.JSONArray
import org.json.JSONObject
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.File
import kotlin.math.abs

class CalibratorTest {

    @Test
    fun testCalibrationNumericalParity() {
        // Read input JSON
        val inPath = "../../engine/calibration/tests/vectors/calibrator_in.json"
        val outPath = "../../engine/calibration/tests/vectors/calibrator_out.json"

        val inFile = File(inPath)
        val outFile = File(outPath)

        assertTrue("Test vector input file not found: ${inFile.absolutePath}", inFile.exists())
        assertTrue("Test vector output file not found: ${outFile.absolutePath}", outFile.exists())

        val inJson = JSONObject(inFile.readText())
        val outJson = JSONObject(outFile.readText())

        val accJson = inJson.getJSONArray("acc")
        val gyroJson = inJson.getJSONArray("gyro")
        val speedJson = inJson.getJSONArray("speed")
        val dt = inJson.getDouble("dt")

        val n = accJson.length()
        val acc = Array(n) { DoubleArray(3) }
        val gyro = Array(n) { DoubleArray(3) }
        val speed = DoubleArray(n)

        for (i in 0 until n) {
            val aRow = accJson.getJSONArray(i)
            val gRow = gyroJson.getJSONArray(i)
            acc[i] = doubleArrayOf(aRow.getDouble(0), aRow.getDouble(1), aRow.getDouble(2))
            gyro[i] = doubleArrayOf(gRow.getDouble(0), gRow.getDouble(1), gRow.getDouble(2))
            speed[i] = speedJson.getDouble(i)
        }

        val calib = CalibrationEngine()
        val success = calib.calibrateFromSession(acc, gyro, speed, dt)

        val expectedSuccess = outJson.getBoolean("success")
        assertEquals("Success flag mismatch", expectedSuccess, success)

        if (expectedSuccess) {
            val expectedScore = outJson.getDouble("alignment_score")
            assertEquals("Alignment score mismatch", expectedScore, calib.alignmentScore, 1e-4)

            val expGyroBias = outJson.getJSONArray("gyro_bias")
            for (i in 0..2) {
                assertEquals("Gyro bias mismatch at $i", expGyroBias.getDouble(i), calib.gyroBias[i], 1e-5)
            }

            val expAccelBias = outJson.getJSONArray("accel_bias")
            for (i in 0..2) {
                assertEquals("Accel bias mismatch at $i", expAccelBias.getDouble(i), calib.accelBias[i], 1e-5)
            }

            val expR = outJson.getJSONArray("R_phone_to_veh")
            for (i in 0..2) {
                val row = expR.getJSONArray(i)
                for (j in 0..2) {
                    assertEquals(
                        "R_phone_to_veh mismatch at [$i][$j]",
                        row.getDouble(j),
                        calib.rPhoneToVeh.get(i, j),
                        1e-5
                    )
                }
            }
        }
    }
}
