package com.example.idr.calibration

import android.content.Context
import android.content.SharedPreferences
import androidx.security.crypto.EncryptedSharedPreferences
import androidx.security.crypto.MasterKeys
import org.ejml.simple.SimpleMatrix
import org.json.JSONArray
import org.json.JSONObject
import kotlin.math.abs

/**
 * Manages persistent storage and retrieval of calibration state on Android
 * using EncryptedSharedPreferences with strict validation and safe fallbacks.
 */
class CalibrationManager(private val context: Context) {

    companion object {
        private const val PREFS_FILE = "idr_calibration_prefs"
        private const val KEY_CALIBRATION_JSON = "calibration_data"
        private const val SCHEMA_VERSION = "1.0"
    }

    private val sharedPreferences: SharedPreferences by lazy {
        try {
            val masterKeyAlias = MasterKeys.getOrCreate(MasterKeys.AES256_GCM_SPEC)
            EncryptedSharedPreferences.create(
                PREFS_FILE,
                masterKeyAlias,
                context,
                EncryptedSharedPreferences.PrefKeyEncryptionScheme.AES256_SIV,
                EncryptedSharedPreferences.PrefValueEncryptionScheme.AES256_GCM
            )
        } catch (e: Exception) {
            // Fallback to standard SharedPreferences in environments without Keystore/AES-GCM support
            context.getSharedPreferences(PREFS_FILE, Context.MODE_PRIVATE)
        }
    }

    /**
     * Save calibration parameters securely.
     */
    fun saveCalibration(engine: CalibrationEngine): Boolean {
        return try {
            val json = JSONObject().apply {
                put("schema_version", SCHEMA_VERSION)
                put("is_calibrated", engine.isCalibrated)
                put("alignment_score", engine.alignmentScore)
                put("gyro_bias", JSONArray(engine.gyroBias))
                put("accel_bias", JSONArray(engine.accelBias))

                val rMat = JSONArray()
                for (i in 0..2) {
                    val row = JSONArray()
                    for (j in 0..2) {
                        row.put(engine.rPhoneToVeh.get(i, j))
                    }
                    rMat.put(row)
                }
                put("R_phone_to_veh", rMat)
            }

            sharedPreferences.edit()
                .putString(KEY_CALIBRATION_JSON, json.toString())
                .apply()
            true
        } catch (e: Exception) {
            false
        }
    }

    /**
     * Load and validate calibration parameters, with safe fallback to uncalibrated defaults.
     */
    fun loadCalibration(engine: CalibrationEngine): Boolean {
        val jsonStr = sharedPreferences.getString(KEY_CALIBRATION_JSON, null) ?: return false
        return try {
            val json = JSONObject(jsonStr)
            if (json.optString("schema_version") != SCHEMA_VERSION) {
                return false
            }

            val isCalibrated = json.optBoolean("is_calibrated", false)
            val alignmentScore = json.optDouble("alignment_score", 0.0)

            // 1. Gyro Bias Validation ([-0.2, 0.2] rad/s)
            val gyroJson = json.optJSONArray("gyro_bias") ?: return false
            if (gyroJson.length() != 3) return false
            val gyroBias = DoubleArray(3)
            for (i in 0..2) {
                val v = gyroJson.getDouble(i)
                if (v.isNaN() || v.isInfinite() || abs(v) > 0.2) return false
                gyroBias[i] = v
            }

            // 2. Accel Bias Validation ([-5.0, 5.0] m/s^2)
            val accelJson = json.optJSONArray("accel_bias") ?: return false
            if (accelJson.length() != 3) return false
            val accelBias = DoubleArray(3)
            for (i in 0..2) {
                val v = accelJson.getDouble(i)
                if (v.isNaN() || v.isInfinite() || abs(v) > 5.0) return false
                accelBias[i] = v
            }

            // 3. Rotation Matrix Validation (Orthonormal, Det > 0)
            val rJson = json.optJSONArray("R_phone_to_veh") ?: return false
            if (rJson.length() != 3) return false
            val rMat = SimpleMatrix(3, 3)
            for (i in 0..2) {
                val row = rJson.getJSONArray(i)
                if (row.length() != 3) return false
                for (j in 0..2) {
                    val v = row.getDouble(j)
                    if (v.isNaN() || v.isInfinite() || abs(v) > 1.0) return false
                    rMat.set(i, j, v)
                }
            }

            // Check orthonormality: ||R * R^T - I||_F < 1e-4
            val diff = rMat.mult(rMat.transpose()).minus(SimpleMatrix.identity(3))
            var frobNormSq = 0.0
            for (i in 0..2) {
                for (j in 0..2) {
                    val valDiff = diff.get(i, j)
                    frobNormSq += valDiff * valDiff
                }
            }
            if (frobNormSq > 1e-4) return false

            // Check determinant
            if (rMat.determinant() < 0.5) return false

            // Apply validated parameters
            engine.isCalibrated = isCalibrated
            engine.alignmentScore = alignmentScore
            engine.gyroBias = gyroBias
            engine.accelBias = accelBias
            engine.rPhoneToVeh = rMat
            true
        } catch (e: Exception) {
            // Default safe fallback
            engine.isCalibrated = false
            engine.alignmentScore = 0.0
            engine.gyroBias = doubleArrayOf(0.0, 0.0, 0.0)
            engine.accelBias = doubleArrayOf(0.0, 0.0, 0.0)
            engine.rPhoneToVeh = SimpleMatrix.identity(3)
            false
        }
    }

    /**
     * Clear persisted calibration.
     */
    fun clearCalibration() {
        sharedPreferences.edit().remove(KEY_CALIBRATION_JSON).apply()
    }
}
