package com.example.idr.nhc_zupt

import org.ejml.simple.SimpleMatrix
import kotlin.math.abs
import kotlin.math.atan
import kotlin.math.atan2

/**
 * Lean-Angle EKF Estimator (N1)
 *
 * Estimates the roll/lean angle of a two-wheeler in real-time
 * using gyro roll rates and specific force / kinematic-based lean inference.
 * Port of engine/nhc_zupt/lean_ekf.py
 */
class LeanAngleEKF(
    val dt: Double = 0.1,
    qPhi: Double = 1e-3,
    qBias: Double = 1e-5,
    val rMeas: Double = 0.1
) {
    // State: [lean_angle (rad), roll_gyro_bias (rad/s)]
    var x: SimpleMatrix = SimpleMatrix(2, 1, true, *doubleArrayOf(0.0, 0.0))

    // Covariance
    var P: SimpleMatrix = SimpleMatrix.identity(2).scale(0.01)

    // Process noise covariance
    private val Q: SimpleMatrix = SimpleMatrix(2, 2, true, *doubleArrayOf(
        qPhi, 0.0,
        0.0, qBias
    ))

    fun predict(gyroY: Double): Double {
        val phi = x.get(0, 0)
        val b = x.get(1, 0)
        val omega = gyroY - b

        // State transition: phi_k+1 = phi_k + omega * dt
        x.set(0, 0, phi + omega * dt)

        // Jacobian F = [[1, -dt], [0, 1]]
        val F = SimpleMatrix(2, 2, true, *doubleArrayOf(
            1.0, -dt,
            0.0, 1.0
        ))

        P = F.mult(P).mult(F.transpose()).plus(Q)

        return x.get(0, 0)
    }

    fun update(accX: Double, accZ: Double, speed: Double = 0.0, gyroZ: Double = 0.0, g: Double = 9.81): Double {
        val phiMeas: Double = if (speed > 1.0 && abs(gyroZ) > 0.05) {
            // Centripetal acceleration balance: tan(phi) = v * omega_z / g
            atan((speed * gyroZ) / g)
        } else {
            // Low speed / stationary: use lateral vs vertical specific force
            atan2(accX, accZ)
        }

        // Measurement Jacobian H = [1, 0]
        val H = SimpleMatrix(1, 2, true, *doubleArrayOf(1.0, 0.0))
        val y = phiMeas - x.get(0, 0)

        // S = H @ P @ H^T + R
        val tempS = H.mult(P).mult(H.transpose())
        val S = tempS.get(0, 0) + rMeas

        // K = P @ H^T / S
        val K = P.mult(H.transpose()).divide(S)

        x = x.plus(K.scale(y))

        val I = SimpleMatrix.identity(2)
        P = (I.minus(K.mult(H))).mult(P)

        return x.get(0, 0)
    }

    fun getLeanAngleDeg(): Double {
        return Math.toDegrees(x.get(0, 0))
    }
}
