package com.example.idr.nhc_zupt

import org.json.JSONArray
import org.json.JSONObject
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.File

class LeanAngleEKFTest {

    @Test
    fun testLeanAngleEKFNumericalParity() {
        val inPath = "../../engine/nhc_zupt/tests/vectors/lean_ekf_in.json"
        val outPath = "../../engine/nhc_zupt/tests/vectors/lean_ekf_out.json"

        val inFile = File(inPath)
        val outFile = File(outPath)

        assertTrue(inFile.exists())
        assertTrue(outFile.exists())

        val inJson = JSONArray(inFile.readText())
        val outJson = JSONArray(outFile.readText())

        val ekf = LeanAngleEKF(dt = 0.1)

        for (i in 0 until inJson.length()) {
            val inp = inJson.getJSONArray(i)
            val gY = inp.getDouble(0)
            val aX = inp.getDouble(1)
            val aZ = inp.getDouble(2)
            val spd = inp.getDouble(3)
            val gZ = inp.getDouble(4)
            val g = inp.getDouble(5)

            val expObj = outJson.getJSONObject(i)
            val expPred = expObj.getDouble("pred_phi")
            val expUpd = expObj.getDouble("upd_phi")

            val actPred = ekf.predict(gY)
            assertEquals("Predict mismatch at step $i", expPred, actPred, 1e-6)

            val actUpd = ekf.update(aX, aZ, spd, gZ, g)
            assertEquals("Update mismatch at step $i", expUpd, actUpd, 1e-6)

            val expState = expObj.getJSONArray("state")
            assertEquals("State[0] mismatch", expState.getDouble(0), ekf.x.get(0, 0), 1e-6)
            assertEquals("State[1] mismatch", expState.getDouble(1), ekf.x.get(1, 0), 1e-6)

            val expCov = expObj.getJSONArray("cov")
            for (r in 0..1) {
                val row = expCov.getJSONArray(r)
                for (c in 0..1) {
                    assertEquals("Cov[$r,$c] mismatch at step $i", row.getDouble(c), ekf.P.get(r, c), 1e-6)
                }
            }
        }
    }
}
