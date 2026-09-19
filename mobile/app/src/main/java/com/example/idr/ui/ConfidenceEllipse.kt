package com.example.idr.ui

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Geometric parameters for a 2D Confidence Ellipse derived from a 2x2 position covariance matrix.
 *
 * @param semiMajorAxisM Semi-major axis length in meters (scaled by confidence factor k).
 * @param semiMinorAxisM Semi-minor axis length in meters (scaled by confidence factor k).
 * @param orientationDeg Orientation of the major axis in degrees (counter-clockwise from East / X-axis).
 * @param confidenceFactor The sigma multiplier used (e.g. 1.0 for 1-sigma, 2.4477 for 95% 2D confidence).
 */
data class ConfidenceEllipse(
    val semiMajorAxisM: Float,
    val semiMinorAxisM: Float,
    val orientationDeg: Float,
    val confidenceFactor: Float = 2.4477f // 95% confidence in 2D (chi-squared with 2 DOF = 5.991, sqrt = 2.4477)
) {
    companion object {
        /**
         * Compute confidence ellipse parameters from a 2x2 position covariance matrix in ENU frame:
         * [ [ P_EE, P_EN ],
         *   [ P_EN, P_NN ] ]
         *
         * @param pEE East variance (m^2)
         * @param pNN North variance (m^2)
         * @param pEN East-North cross-covariance (m^2)
         * @param k Confidence scaling factor (default: 2.4477 for 95% confidence)
         */
        fun fromCovariance(
            pEE: Double,
            pNN: Double,
            pEN: Double,
            k: Float = 2.4477f
        ): ConfidenceEllipse {
            // Eigenvalues of 2x2 symmetric matrix
            val avg = (pEE + pNN) / 2.0
            val diff = (pEE - pNN) / 2.0
            val disc = sqrt(diff * diff + pEN * pEN)

            val lambda1 = (avg + disc).coerceAtLeast(1e-6)
            val lambda2 = (avg - disc).coerceAtLeast(1e-6)

            val majorM = (k * sqrt(lambda1)).toFloat()
            val minorM = (k * sqrt(lambda2)).toFloat()

            // Angle of eigenvector corresponding to lambda1
            // theta = 0.5 * atan2(2 * P_EN, P_EE - P_NN) in radians
            val thetaRad = 0.5 * atan2(2.0 * pEN, pEE - pNN)
            val thetaDeg = Math.toDegrees(thetaRad).toFloat()

            return ConfidenceEllipse(
                semiMajorAxisM = majorM,
                semiMinorAxisM = minorM,
                orientationDeg = thetaDeg,
                confidenceFactor = k
            )
        }
    }
}
