package com.example.idr.fusion

import org.json.JSONArray
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.File

class AICorrectionModuleTest {

    @Test
    fun testAiCorrectorNumericalParity() {
        val inPath = "../../engine/fusion/tests/vectors/ai_corrector_in.json"
        val outPath = "../../engine/fusion/tests/vectors/ai_corrector_out.json"

        val inFile = File(inPath)
        val outFile = File(outPath)

        assertTrue(inFile.exists())
        assertTrue(outFile.exists())

        val inJson = JSONArray(inFile.readText())
        val outJson = JSONArray(outFile.readText())

        val aiCorrector = AICorrectionModule(windowSize = 5, baseSigmaAi = 1.0)

        for (i in 0 until inJson.length()) {
            val inp = inJson.getJSONObject(i)
            val accArr = inp.getJSONArray("acc")
            val gyroArr = inp.getJSONArray("gyro")

            val acc = doubleArrayOf(accArr.getDouble(0), accArr.getDouble(1), accArr.getDouble(2))
            val gyro = doubleArrayOf(gyroArr.getDouble(0), gyroArr.getDouble(1), gyroArr.getDouble(2))

            val (actSpeed, actSigma, actQScale) = aiCorrector.processImuSample(acc, gyro)

            val expObj = outJson.getJSONObject(i)
            // Python returns None as null in JSON, so we handle it.
            // But let's verify if the null in JSON is parsed correctly.

            val expSpeed = if (expObj.isNull("ai_speed")) null else expObj.getDouble("ai_speed")
            val expSigma = expObj.getDouble("sigma_ai")
            val expQScale = expObj.getDouble("q_scale")

            assertEquals("Speed mismatch at step $i", expSpeed, actSpeed)
            assertEquals("Sigma mismatch at step $i", expSigma, actSigma, 1e-4)
            assertEquals("QScale mismatch at step $i", expQScale, actQScale, 1e-4)
        }
    }
}
