package com.example.idr.nhc_zupt

import org.json.JSONArray
import org.json.JSONObject
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.File

class ConstrainedINSTest {

    @Test
    fun testConstrainedINSNumericalParity() {
        val inPath = "../../engine/nhc_zupt/tests/vectors/constrained_ins_in.json"
        val outPath = "../../engine/nhc_zupt/tests/vectors/constrained_ins_out.json"

        val inFile = File(inPath)
        val outFile = File(outPath)

        assertTrue(inFile.exists())
        assertTrue(outFile.exists())

        val inJson = JSONArray(inFile.readText())
        val outJson = JSONArray(outFile.readText())

        val cins = ConstrainedINS(dt = 0.1)

        for (i in 0 until inJson.length()) {
            val inp = inJson.getJSONObject(i)
            val accArr = inp.getJSONArray("acc_veh")
            val gyroArr = inp.getJSONArray("gyro_veh")
            val vArr = inp.getJSONArray("v_veh")
            val vehicleType = inp.getString("vehicle_type")
            val leanAngleRad = inp.getDouble("lean_angle_rad")

            val acc = doubleArrayOf(accArr.getDouble(0), accArr.getDouble(1), accArr.getDouble(2))
            val gyro = doubleArrayOf(gyroArr.getDouble(0), gyroArr.getDouble(1), gyroArr.getDouble(2))
            val v = doubleArrayOf(vArr.getDouble(0), vArr.getDouble(1), vArr.getDouble(2))

            val expObj = outJson.getJSONObject(i)
            val expStopped = expObj.getBoolean("is_stopped")
            val expVConst = expObj.getJSONArray("v_constrained")

            val actStopped = cins.isStopped(acc, gyro, v)
            assertEquals("Stopped detection mismatch at step $i", expStopped, actStopped)

            val actVConst = cins.constrain(acc, gyro, v, vehicleType, leanAngleRad)
            for (j in 0..2) {
                assertEquals("v_constrained[$j] mismatch at step $i", expVConst.getDouble(j), actVConst[j], 1e-6)
            }
        }
    }
}