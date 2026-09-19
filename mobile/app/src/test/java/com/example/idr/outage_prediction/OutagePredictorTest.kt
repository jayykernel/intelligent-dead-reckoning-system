package com.example.idr.outage_prediction

import org.json.JSONArray
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.File

class OutagePredictorTest {

    @Test
    fun testOutagePredictorNumericalParity() {
        val inPath = "../../engine/outage_prediction/tests/vectors/outage_predictor_in.json"
        val outPath = "../../engine/outage_prediction/tests/vectors/outage_predictor_out.json"

        val inFile = File(inPath)
        val outFile = File(outPath)

        assertTrue(inFile.exists())
        assertTrue(outFile.exists())

        val inJson = JSONArray(inFile.readText())
        val outJson = JSONArray(outFile.readText())

        val predictor = OutagePredictor(windowSizeSec = 3.0, dt = 1.0)

        for (i in 0 until inJson.length()) {
            val inp = inJson.getJSONObject(i)
            val cn0 = inp.getDouble("cn0")
            val sat = inp.getInt("sat")
            val acc = inp.getDouble("acc")

            val actTrust = predictor.update(avgCn0 = cn0, satCount = sat, accuracyM = acc)

            val expObj = outJson.getJSONObject(i)
            val expTrust = expObj.getDouble("trust")

            println("Step $i: exp=$expTrust, act=$actTrust")
            assertEquals("Trust mismatch at step $i", expTrust, actTrust, 1e-5)
        }
    }
}
