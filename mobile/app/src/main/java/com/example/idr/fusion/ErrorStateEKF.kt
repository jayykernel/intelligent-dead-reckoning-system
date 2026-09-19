package com.example.idr.fusion

import org.ejml.simple.SimpleMatrix
import kotlin.math.abs
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * 15-State Error-State Extended Kalman Filter (ES-EKF) for GNSS+INS Fusion.
 * Port of engine/fusion/ekf.py
 */
class ErrorStateEKF(
    val dt: Double = 0.1,
    val sigmaAcc: Double = 0.2,
    val sigmaGyro: Double = 0.02,
    val sigmaAccBias: Double = 0.001,
    val sigmaGyroBias: Double = 0.0001
) {
    // Nominal states
    var p: DoubleArray = doubleArrayOf(0.0, 0.0, 0.0) // Position ENU
    var v: DoubleArray = doubleArrayOf(0.0, 0.0, 0.0) // Velocity ENU
    var q: DoubleArray = doubleArrayOf(1.0, 0.0, 0.0, 0.0) // Quaternion [qw, qx, qy, qz]
    var bA: DoubleArray = doubleArrayOf(0.0, 0.0, 0.0) // Accel bias
    var bG: DoubleArray = doubleArrayOf(0.0, 0.0, 0.0) // Gyro bias

    // 15x15 Error state covariance
    var P: SimpleMatrix = SimpleMatrix.identity(15)

    // Gravity in navigation frame (ENU: Z is Up)
    val gNav: DoubleArray = doubleArrayOf(0.0, 0.0, -9.80665)

    init {
        // Initial uncertainties
        setBlockDiag(0, 3, 5.0 * 5.0)
        setBlockDiag(3, 3, 1.0 * 1.0)
        val rad5 = Math.toRadians(5.0)
        setBlockDiag(6, 3, rad5 * rad5)
        setBlockDiag(9, 3, 0.1 * 0.1)
        setBlockDiag(12, 3, 0.01 * 0.01)
    }

    private fun setBlockDiag(startIdx: Int, size: Int, value: Double) {
        for (i in 0 until size) {
            P.set(startIdx + i, startIdx + i, value)
        }
    }

    fun setInitialState(
        p0: DoubleArray,
        v0: DoubleArray,
        q0: DoubleArray,
        bA0: DoubleArray? = null,
        bG0: DoubleArray? = null
    ) {
        p = p0.copyOf()
        v = v0.copyOf()
        val qNorm = norm4(q0)
        q = DoubleArray(4) { q0[it] / qNorm }
        if (bA0 != null) bA = bA0.copyOf()
        if (bG0 != null) bG = bG0.copyOf()
    }

    companion object {
        fun quatToRot(q: DoubleArray): SimpleMatrix {
            val qw = q[0]
            val qx = q[1]
            val qy = q[2]
            val qz = q[3]

            val R = SimpleMatrix(3, 3)
            R.set(0, 0, 1.0 - 2.0 * (qy * qy + qz * qz))
            R.set(0, 1, 2.0 * (qx * qy - qw * qz))
            R.set(0, 2, 2.0 * (qx * qz + qw * qy))

            R.set(1, 0, 2.0 * (qx * qy + qw * qz))
            R.set(1, 1, 1.0 - 2.0 * (qx * qx + qz * qz))
            R.set(1, 2, 2.0 * (qy * qz - qw * qx))

            R.set(2, 0, 2.0 * (qx * qz - qw * qy))
            R.set(2, 1, 2.0 * (qy * qz + qw * qx))
            R.set(2, 2, 1.0 - 2.0 * (qx * qx + qy * qy))

            return R
        }

        fun skewSymmetric(v: DoubleArray): SimpleMatrix {
            val S = SimpleMatrix(3, 3)
            S.set(0, 1, -v[2])
            S.set(0, 2, v[1])
            S.set(1, 0, v[2])
            S.set(1, 2, -v[0])
            S.set(2, 0, -v[1])
            S.set(2, 1, v[0])
            return S
        }

        private val CHI2_TABLE_099 = doubleArrayOf(
            0.0,
            6.6348966010212145, 9.21034037197618, 11.344866730144373,
            13.276704135987622, 15.08627246938899, 16.811893829770927,
            18.475306906582357, 20.090235029663233, 21.665994333461924,
            23.209251158954356, 24.724970311318277, 26.216967305535853,
            27.68824961045705, 29.141237740672796, 30.57791416689249
        )
    }

    fun predict(accRaw: DoubleArray, gyroRaw: DoubleArray, dtVal: Double? = null, qScale: Double = 1.0) {
        val dtStep = dtVal ?: dt

        // 1. Bias-corrected IMU measurements
        val accCorr = doubleArrayOf(accRaw[0] - bA[0], accRaw[1] - bA[1], accRaw[2] - bA[2])
        val gyroCorr = doubleArrayOf(gyroRaw[0] - bG[0], gyroRaw[1] - bG[1], gyroRaw[2] - bG[2])

        // 2. Update nominal orientation
        val rotVec = doubleArrayOf(gyroCorr[0] * dtStep, gyroCorr[1] * dtStep, gyroCorr[2] * dtStep)
        val angle = norm3(rotVec)
        val dq = if (angle < 1e-12) {
            doubleArrayOf(1.0 - angle * angle / 8.0, rotVec[0] / 2.0, rotVec[1] / 2.0, rotVec[2] / 2.0)
        } else {
            val axis = doubleArrayOf(rotVec[0] / angle, rotVec[1] / angle, rotVec[2] / angle)
            val halfAngle = angle / 2.0
            val sinHalf = sin(halfAngle)
            doubleArrayOf(cos(halfAngle), axis[0] * sinHalf, axis[1] * sinHalf, axis[2] * sinHalf)
        }

        // Quaternion multiplication: q_new = q * dq
        val qw = q[0]
        val qx = q[1]
        val qy = q[2]
        val qz = q[3]
        val dw = dq[0]
        val dx = dq[1]
        val dy = dq[2]
        val dz = dq[3]

        val qNew = doubleArrayOf(
            qw * dw - qx * dx - qy * dy - qz * dz,
            qw * dx + qx * dw + qy * dz - qz * dy,
            qw * dy - qx * dz + qy * dw + qz * dx,
            qw * dz + qx * dy - qy * dx + qz * dw
        )
        val qNorm = norm4(qNew)
        q = DoubleArray(4) { qNew[it] / qNorm }

        // 3. Specific force in Nav frame
        val R = quatToRot(q)
        val accCorrMat = SimpleMatrix(3, 1, true, *accCorr)
        val fNavMat = R.mult(accCorrMat)
        val aNav = doubleArrayOf(
            fNavMat.get(0, 0) + gNav[0],
            fNavMat.get(1, 0) + gNav[1],
            fNavMat.get(2, 0) + gNav[2]
        )

        // 4. Integrate Position and Velocity
        for (i in 0..2) {
            p[i] += v[i] * dtStep + 0.5 * aNav[i] * dtStep * dtStep
            v[i] += aNav[i] * dtStep
        }

        // 5. Continuous-to-Discrete Error State Transition Matrix F
        val F = SimpleMatrix.identity(15)

        // F[0:3, 3:6] = I * dt
        for (i in 0..2) F.set(i, 3 + i, dtStep)

        // F[0:3, 6:9] = -0.5 * R * skew(accCorr) * dt^2
        val skewAcc = skewSymmetric(accCorr)
        val fPosAtt = R.mult(skewAcc).scale(-0.5 * dtStep * dtStep)
        for (r in 0..2) for (c in 0..2) F.set(r, 6 + c, fPosAtt.get(r, c))

        // F[0:3, 9:12] = -0.5 * R * dt^2
        val fPosBa = R.scale(-0.5 * dtStep * dtStep)
        for (r in 0..2) for (c in 0..2) F.set(r, 9 + c, fPosBa.get(r, c))

        // F[3:6, 6:9] = -R * skew(accCorr) * dt
        val fVelAtt = R.mult(skewAcc).scale(-dtStep)
        for (r in 0..2) for (c in 0..2) F.set(3 + r, 6 + c, fVelAtt.get(r, c))

        // F[3:6, 9:12] = -R * dt
        val fVelBa = R.scale(-dtStep)
        for (r in 0..2) for (c in 0..2) F.set(3 + r, 9 + c, fVelBa.get(r, c))

        // F[6:9, 6:9] = I - skew(gyroCorr) * dt
        val skewGyro = skewSymmetric(gyroCorr)
        for (r in 0..2) {
            for (c in 0..2) {
                val delta = if (r == c) 1.0 else 0.0
                F.set(6 + r, 6 + c, delta - skewGyro.get(r, c) * dtStep)
            }
        }

        // F[6:9, 12:15] = -I * dt
        for (i in 0..2) F.set(6 + i, 12 + i, -dtStep)

        // 6. Process Noise Covariance Q
        val Q = SimpleMatrix(15, 15)
        val qPos = 0.5 * (sigmaAcc * qScale) * dtStep * dtStep
        val qVel = (sigmaAcc * qScale) * dtStep

        val gyroMag = norm3(gyroCorr)
        val adaptiveSigmaGyro = sigmaGyro * qScale + 2.0 * gyroMag
        val qAtt = adaptiveSigmaGyro * dtStep

        val qBa = sigmaAccBias * sqrt(dtStep)
        val qBg = sigmaGyroBias * sqrt(dtStep)

        for (i in 0..2) {
            Q.set(i, i, qPos * qPos)
            Q.set(3 + i, 3 + i, qVel * qVel)
            Q.set(6 + i, 6 + i, qAtt * qAtt)
            Q.set(9 + i, 9 + i, qBa * qBa)
            Q.set(12 + i, 12 + i, qBg * qBg)
        }

        // 7. Covariance propagation: P = F @ P @ F.T + Q
        P = F.mult(P).mult(F.transpose()).plus(Q)

        // Enforce symmetry
        P = P.plus(P.transpose()).scale(0.5)
    }

    fun update(
        z: DoubleArray,
        hX: DoubleArray,
        H: SimpleMatrix,
        RCov: SimpleMatrix,
        updateType: String = "GNSS_POS"
    ): Triple<Boolean, Double, Double> {
        val m = z.size

        // 1. Innovation vector
        val y = DoubleArray(m) { z[it] - hX[it] }
        if (updateType == "MAG_HEADING" || updateType == "GNSS_HEADING") {
            // Wrap angular innovation to [-pi, pi]
            y[0] = (y[0] + Math.PI) % (2 * Math.PI) - Math.PI
        }

        // 2. Innovation Covariance: S = H @ P @ H.T + R_cov
        var S = H.mult(P).mult(H.transpose()).plus(RCov)
        S = S.plus(S.transpose()).scale(0.5)

        // 3. NIS
        val yMat = SimpleMatrix(m, 1, true, *y)
        val SInv = S.invert()
        val nisMat = yMat.transpose().mult(SInv).mult(yMat)
        val nis = nisMat.get(0, 0)

        val chi2Thresh = if (m in 1..15) CHI2_TABLE_099[m] else 15.0
        val passed = nis <= chi2Thresh

        if (!passed) {
            return Triple(false, nis, chi2Thresh)
        }

        // 5. Kalman Gain: K = P @ H.T @ S_inv (15, M)
        val K = P.mult(H.transpose()).mult(SInv)

        // 6. Error state correction: delta_x = K @ y
        val deltaXMat = K.mult(yMat)
        val deltaX = DoubleArray(15) { deltaXMat.get(it, 0) }

        // 7. Joseph form covariance update: P = (I - K H) P (I - K H)^T + K R K^T
        val I15 = SimpleMatrix.identity(15)
        val IMinusKH = I15.minus(K.mult(H))
        P = IMinusKH.mult(P).mult(IMinusKH.transpose()).plus(K.mult(RCov).mult(K.transpose()))
        P = P.plus(P.transpose()).scale(0.5)

        // 8. Inject error states into nominal state
        injectErrorState(deltaX)

        return Triple(true, nis, chi2Thresh)
    }

    private fun injectErrorState(deltaX: DoubleArray) {
        for (i in 0..2) {
            p[i] += deltaX[i]
            v[i] += deltaX[3 + i]
        }

        val deltaTheta = doubleArrayOf(deltaX[6], deltaX[7], deltaX[8])
        val dAngle = norm3(deltaTheta)
        val dq = if (dAngle < 1e-12) {
            doubleArrayOf(1.0, 0.5 * deltaTheta[0], 0.5 * deltaTheta[1], 0.5 * deltaTheta[2])
        } else {
            val axis = doubleArrayOf(deltaTheta[0] / dAngle, deltaTheta[1] / dAngle, deltaTheta[2] / dAngle)
            val half = dAngle / 2.0
            val sinHalf = sin(half)
            doubleArrayOf(cos(half), axis[0] * sinHalf, axis[1] * sinHalf, axis[2] * sinHalf)
        }

        val qw = q[0]
        val qx = q[1]
        val qy = q[2]
        val qz = q[3]
        val dw = dq[0]
        val dx = dq[1]
        val dy = dq[2]
        val dz = dq[3]

        val qNew = doubleArrayOf(
            qw * dw - qx * dx - qy * dy - qz * dz,
            qw * dx + qx * dw + qy * dz - qz * dy,
            qw * dy - qx * dz + qy * dw + qz * dx,
            qw * dz + qx * dy - qy * dx + qz * dw
        )
        val qNorm = norm4(qNew)
        q = DoubleArray(4) { qNew[it] / qNorm }

        for (i in 0..2) {
            bA[i] += deltaX[9 + i]
            bG[i] += deltaX[12 + i]
        }
    }

    fun updateGnssPosition(
        pGnssEnu: DoubleArray,
        sigmaPos: Double = 3.0
    ): Triple<Boolean, Double, Double> {
        val H = SimpleMatrix(3, 15)
        for (i in 0..2) H.set(i, i, 1.0)
        val RCov = SimpleMatrix.identity(3).scale(sigmaPos * sigmaPos)
        return update(pGnssEnu, p, H, RCov, updateType = "GNSS_POS")
    }

    fun updateGnssVelocity(
        vGnssEnu: DoubleArray,
        sigmaVel: Double = 0.5
    ): Triple<Boolean, Double, Double> {
        val H = SimpleMatrix(3, 15)
        for (i in 0..2) H.set(i, 3 + i, 1.0)
        val RCov = SimpleMatrix.identity(3).scale(sigmaVel * sigmaVel)
        return update(vGnssEnu, v, H, RCov, updateType = "GNSS_VEL")
    }

    fun updateHeading(
        headingRad: Double,
        sigmaHeading: Double = Math.toRadians(5.0),
        source: String = "MAG_HEADING"
    ): Triple<Boolean, Double, Double> {
        val R = quatToRot(q)
        val currentYaw = atan2(R.get(0, 1), R.get(1, 1))

        val z = doubleArrayOf(headingRad)
        val hX = doubleArrayOf(currentYaw)

        val H = SimpleMatrix(1, 15)
        H.set(0, 8, -1.0) // delta_theta_z

        val RCov = SimpleMatrix(1, 1, true, sigmaHeading * sigmaHeading)
        return update(z, hX, H, RCov, updateType = source)
    }

    fun updateAiForwardSpeed(
        speedFwd: Double,
        sigmaSpeed: Double = 1.0
    ): Triple<Boolean, Double, Double> {
        val R = quatToRot(q)
        val yAxisNav = doubleArrayOf(R.get(0, 1), R.get(1, 1), R.get(2, 1))
        val vFwdEst = yAxisNav[0] * v[0] + yAxisNav[1] * v[1] + yAxisNav[2] * v[2]

        val z = doubleArrayOf(speedFwd)
        val hX = doubleArrayOf(vFwdEst)

        val H = SimpleMatrix(1, 15)
        for (i in 0..2) H.set(0, 3 + i, yAxisNav[i])

        val crossProduct = doubleArrayOf(
            yAxisNav[1] * v[2] - yAxisNav[2] * v[1],
            yAxisNav[2] * v[0] - yAxisNav[0] * v[2],
            yAxisNav[0] * v[1] - yAxisNav[1] * v[0]
        )
        for (i in 0..2) H.set(0, 6 + i, crossProduct[i])

        val RCov = SimpleMatrix(1, 1, true, sigmaSpeed * sigmaSpeed)
        return update(z, hX, H, RCov, updateType = "AI_SPEED")
    }

    fun updateZupt(
        sigmaZupt: Double = 0.05
    ): Triple<Boolean, Double, Double> {
        val z = doubleArrayOf(0.0, 0.0, 0.0)
        val hX = v.copyOf()
        val H = SimpleMatrix(3, 15)
        for (i in 0..2) H.set(i, 3 + i, 1.0)
        val RCov = SimpleMatrix.identity(3).scale(sigmaZupt * sigmaZupt)
        return update(z, hX, H, RCov, updateType = "ZUPT")
    }

    fun updateNhc(
        vehicleType: String = "car",
        leanAngleRad: Double = 0.0,
        sigmaNhcX: Double = 0.2,
        sigmaNhcZ: Double = 0.2
    ): Triple<Boolean, Double, Double> {
        val R = quatToRot(q)
        val vMat = SimpleMatrix(3, 1, true, *v)
        val vVehMat = R.transpose().mult(vMat)
        val vVeh = doubleArrayOf(vVehMat.get(0, 0), vVehMat.get(1, 0), vVehMat.get(2, 0))

        val M: SimpleMatrix
        val N: SimpleMatrix
        val vMeas: DoubleArray

        if (vehicleType == "two_wheeler" && abs(leanAngleRad) > 1e-4) {
            val phi = leanAngleRad
            val c = cos(phi)
            val s = sin(phi)
            val rY = SimpleMatrix(3, 3, true, *doubleArrayOf(
                c, 0.0, s,
                0.0, 1.0, 0.0,
                -s, 0.0, c
            ))
            M = rY.mult(R.transpose())
            N = rY.mult(skewSymmetric(vVeh))
            val vMeasMat = rY.mult(vVehMat)
            vMeas = doubleArrayOf(vMeasMat.get(0, 0), vMeasMat.get(1, 0), vMeasMat.get(2, 0))
        } else {
            M = R.transpose()
            N = skewSymmetric(vVeh)
            vMeas = vVeh
        }

        val z = doubleArrayOf(0.0, 0.0)
        val hX = doubleArrayOf(vMeas[0], vMeas[2])

        val H = SimpleMatrix(2, 15)
        for (c in 0..2) {
            H.set(0, 3 + c, M.get(0, c))
            H.set(1, 3 + c, M.get(2, c))
            H.set(0, 6 + c, N.get(0, c))
            H.set(1, 6 + c, N.get(2, c))
        }

        val RCov = SimpleMatrix(2, 2, true, *doubleArrayOf(
            sigmaNhcX * sigmaNhcX, 0.0,
            0.0, sigmaNhcZ * sigmaNhcZ
        ))

        return update(z, hX, H, RCov, updateType = "NHC")
    }

    fun getPositionCovariance2d(): SimpleMatrix {
        val cov2d = SimpleMatrix(2, 2)
        cov2d.set(0, 0, P.get(0, 0))
        cov2d.set(0, 1, P.get(0, 1))
        cov2d.set(1, 0, P.get(1, 0))
        cov2d.set(1, 1, P.get(1, 1))
        return cov2d
    }

    fun getEulerAnglesDeg(): Triple<Double, Double, Double> {
        val R = quatToRot(q)
        val r21 = R.get(2, 1).coerceIn(-1.0, 1.0)
        val pitch = asin(-r21)
        val roll = atan2(R.get(2, 0), R.get(2, 2))
        val yaw = atan2(R.get(0, 1), R.get(1, 1))
        return Triple(Math.toDegrees(roll), Math.toDegrees(pitch), Math.toDegrees(yaw))
    }

    private fun norm3(v: DoubleArray): Double = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    private fun norm4(q: DoubleArray): Double = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
}
