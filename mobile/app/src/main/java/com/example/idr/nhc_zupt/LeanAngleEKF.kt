package com.example.idr.nhc_zupt

import org.ejml.simple.SimpleMatrix
import kotlin.math.abs
import kotlin.math.atan
import kotlin.math.atan2
import kotlin.math.max

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

    private fun validateState() {
        // Check if state contains NaN or Inf
        if (!x.get(0, 0).isFinite() || !x.get(1, 0).isFinite()) {
            throw IllegalArgumentException("NaN/Inf in LeanAngleEKF state")
        }
        // Check if covariance contains NaN or Inf
        for (i in 0 until 2) {
            for (j in 0 until 2) {
                if (!P.get(i, j).isFinite()) {
                    throw IllegalArgumentException("NaN/Inf in LeanAngleEKF covariance")
                }
            }
        }
    }

    private fun ensurePositiveDefinite(epsilon: Double = 1e-9) {
        // Check if covariance contains NaN or Inf before processing
        for (i in 0 until 2) {
            for (j in 0 until 2) {
                if (!P.get(i, j).isFinite()) {
                    throw IllegalArgumentException("NaN/Inf in LeanAngleEKF covariance before positive-definite check")
                }
            }
        }

        // Symmetrize: P = 0.5 * (P + P^T)
        val Ptrans = P.transpose()
        val Psym = SimpleMatrix(2, 2)
        for (i in 0 until 2) {
            for (j in 0 until 2) {
                Psym.set(i, j, 0.5 * (P.get(i, j) + Ptrans.get(i, j)))
            }
        }
        P = Psym

        // Eigenvalue decomposition of symmetric matrix
        val eigen = P.eig()
        if (!eigen.hasEigenvector) {
            // Fallback to diagonal matrix if eigen decomposition fails
            P = SimpleMatrix.identity(2).scale(epsilon)
            return
        }

        val eigenvalues = eigen.getRealEigenvalues()
        val eigenvectors = eigen.getEigenVector

        // Eigenvalue flooring: clip eigenvalues to minimum epsilon
        val clippedEigenvalues = DoubleArray(2) { max(eigenvalues[i], epsilon) }

        // Reconstruct: P = V * D * V^T
        val V = eigenvectors
        val D = SimpleMatrix(2, 2, true, *clippedEigenvalues)
        val VD = V.mult(D)
        P = VD.mult(V.transpose())

        // Ensure symmetry again
        val Ptrans2 = P.transpose()
        val Psym2 = SimpleMatrix(2, 2)
        for (i in 0 until 2) {
            for (j in 0 until 2) {
                Psym2.set(i, j, 0.5 * (P.get(i, j) + Ptrans2.get(i, j)))
            }
        }
        P = Psym2
    }

    fun predict(gyroY: Double): Double {
        if (!gyroY.isFinite()) {
            throw IllegalArgumentException("NaN/Inf gyro_y input to LeanAngleEKF.predict")
        }

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
        ensurePositiveDefinite()
        validateState()

        return x.get(0, 0)
    }

    fun update(accX: Double, accZ: Double, speed: Double = 0.0, gyroZ: Double = 0.0, g: Double = 9.81): Double {
        if (!accX.isFinite() || !accZ.isFinite() || !speed.isFinite() || !gyroZ.isFinite() || !g.isFinite()) {
            throw IllegalArgumentException("NaN/Inf input to LeanAngleEKF.update")
        }

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

        // Joseph form covariance update: P = (I - K H) P (I - K H)^T + K R K^T
        val I = SimpleMatrix.identity(2)
        val I_KH = I.minus(K.mult(H))
        val RK_cov = SimpleMatrix(1, 1, true, *doubleArrayOf(rMeas))
        val K_R_Kt = K.mult(RK_cov).mult(K.transpose())
        P = I_KH.mult(P).mult(I_KH.transpose()).plus(K_R_Kt)

        ensurePositiveDefinite()
        validateState()

        return x.get(0, 0)
    }

    fun getLeanAngleDeg(): Double {
        return Math.toDegrees(x.get(0, 0))
    }
}
