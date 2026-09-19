package com.example.idr.ui

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.DashPathEffect
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import android.util.AttributeSet
import android.util.Log
import android.view.View

/**
 * Custom Canvas View for rendering the 2D Real-Time Confidence Ellipse (N4).
 *
 * Directly visualizes the 15-state ES-EKF position covariance matrix:
 * - Grows smoothly as dead-reckoning drift accumulates.
 * - Collapses immediately on NIS-passing GNSS reacquisition.
 * - Stays visibly large during unresolved long-outage drift without false contraction.
 */
class ConfidenceEllipseView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    private val TAG = "IDR_EllipseView"

    // Current State Parameters
    private var ellipse: ConfidenceEllipse = ConfidenceEllipse(semiMajorAxisM = 2.0f, semiMinorAxisM = 1.5f, orientationDeg = 0.0f)
    private var mode: String = "GNSS_AIDED"
    private var trustScore: Float = 1.0f
    private var headingDeg: Float = 0.0f
    private var nisPassed: Boolean = true

    // Pixels per meter scale (e.g., 10 pixels = 1 meter for close inspection)
    private var pixelsPerMeter: Float = 8.0f

    // Drawing Paints
    private val backgroundPaint = Paint().apply {
        style = Paint.Style.FILL
        color = Color.WHITE
    }

    private val ellipseFillPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
        color = Color.argb(45, 46, 125, 50) // Default green fill
    }
    private val ellipseStrokePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 4.0f
        color = Color.rgb(46, 125, 50) // Default green stroke
    }
    private val oneSigmaStrokePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 2.0f
        color = Color.argb(120, 46, 125, 50)
        pathEffect = DashPathEffect(floatArrayOf(10f, 10f), 0f)
    }

    private val vehiclePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
        color = Color.rgb(30, 136, 229) // Blue marker
    }

    private val vehicleHeadingPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 4.0f
        color = Color.rgb(21, 101, 192)
    }

    private val gridPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 1.5f
        color = Color.argb(50, 150, 150, 150)
    }

    private val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(20, 20, 20) // Solid dark text
        textSize = 38.0f
        typeface = android.graphics.Typeface.DEFAULT_BOLD
    }

    private val ovalRect = RectF()
    private val path = Path()

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        Log.d(TAG, "onSizeChanged: width=$w, height=$h")
        if (w > 0) {
            val maxAxisM = ellipse.semiMajorAxisM.coerceAtLeast(1.0f)
            this.pixelsPerMeter = (w / (maxAxisM * 3.2f)).coerceIn(1.5f, 25.0f)
            invalidate()
        }
    }

    /**
     * Update the visual state directly from the fusion engine output.
     */
    fun updateState(
        pEE: Double,
        pNN: Double,
        pEN: Double,
        currentMode: String,
        trust: Float,
        heading: Float,
        nisPassed: Boolean
    ) {
        Log.d(TAG, "updateState: mode=$currentMode, trust=$trust, nisPassed=$nisPassed, pEE=$pEE, pNN=$pNN")
        this.ellipse = ConfidenceEllipse.fromCovariance(pEE, pNN, pEN)
        this.mode = currentMode
        this.trustScore = trust
        this.headingDeg = heading
        this.nisPassed = nisPassed

        // Use Phase 8/9 thresholds exactly: <0.2 drop, >0.8 recover.
        // Amber bridges the gap during transition logic. Purple forces rejected rendering.
        if (!nisPassed && currentMode == "GNSS_AIDED") {
            // NIS Rejected (Purple)
            ellipseFillPaint.color = Color.argb(45, 128, 0, 128)
            ellipseStrokePaint.color = Color.rgb(128, 0, 128)
            oneSigmaStrokePaint.color = Color.argb(120, 128, 0, 128)
        } else if (currentMode == "GNSS_AIDED") {
            if (trust < 0.8f) {
                // Amber (Trust dropping toward 0.2, or recovering but hasn't breached 0.8 yet)
                ellipseFillPaint.color = Color.argb(45, 255, 165, 0)
                ellipseStrokePaint.color = Color.rgb(255, 140, 0)
                oneSigmaStrokePaint.color = Color.argb(120, 255, 140, 0)
            } else {
                // Green (Trust > 0.8, normal operation)
                ellipseFillPaint.color = Color.argb(45, 46, 125, 50)
                ellipseStrokePaint.color = Color.rgb(46, 125, 50)
                oneSigmaStrokePaint.color = Color.argb(120, 46, 125, 50)
            }
        } else {
            // PURE_DEAD_RECKONING (Red)
            ellipseFillPaint.color = Color.argb(55, 198, 40, 40)
            ellipseStrokePaint.color = Color.rgb(198, 40, 40)
            oneSigmaStrokePaint.color = Color.argb(130, 198, 40, 40)
        }

        // Auto-scale pixels per meter so the ellipse always fits nicely on screen
        if (width > 0) {
            val maxAxisM = ellipse.semiMajorAxisM.coerceAtLeast(1.0f)
            val targetPpm = (width / (maxAxisM * 3.2f)).coerceIn(1.5f, 25.0f)
            this.pixelsPerMeter = targetPpm
        }

        invalidate() // Request redraw
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)

        if (width == 0 || height == 0) return

        // 0. Ensure solid white canvas background
        canvas.drawRect(0f, 0f, width.toFloat(), height.toFloat(), backgroundPaint)

        val cx = width / 2.0f
        val cy = height / 2.0f

        // 1. Draw concentric distance reference rings
        val rings = floatArrayOf(2f, 5f, 10f, 20f, 40f)
        for (r in rings) {
            val radiusPx = r * pixelsPerMeter
            if (radiusPx < width / 2.0f) {
                canvas.drawCircle(cx, cy, radiusPx, gridPaint)
            }
        }

        // 2. Draw 95% Confidence Ellipse
        canvas.save()
        canvas.translate(cx, cy)
        canvas.rotate(-ellipse.orientationDeg)

        val aPx = ellipse.semiMajorAxisM * pixelsPerMeter
        val bPx = ellipse.semiMinorAxisM * pixelsPerMeter

        ovalRect.set(-aPx, -bPx, aPx, bPx)
        canvas.drawOval(ovalRect, ellipseFillPaint)
        canvas.drawOval(ovalRect, ellipseStrokePaint)

        // 3. Draw 1-Sigma inner ellipse
        val a1Px = (ellipse.semiMajorAxisM / ellipse.confidenceFactor) * pixelsPerMeter
        val b1Px = (ellipse.semiMinorAxisM / ellipse.confidenceFactor) * pixelsPerMeter
        ovalRect.set(-a1Px, -b1Px, a1Px, b1Px)
        canvas.drawOval(ovalRect, oneSigmaStrokePaint)

        canvas.restore()

        // 4. Draw Vehicle Pointer
        canvas.save()
        canvas.translate(cx, cy)
        canvas.rotate(headingDeg)

        path.reset()
        path.moveTo(0f, -22f)
        path.lineTo(-14f, 16f)
        path.lineTo(0f, 10f)
        path.lineTo(14f, 16f)
        path.close()

        canvas.drawPath(path, vehiclePaint)
        canvas.drawPath(path, vehicleHeadingPaint)
        canvas.restore()

        // 5. Render HUD Overlay Metrics
        val hudTextY = 50.0f
        canvas.drawText("Mode: $mode", 30.0f, hudTextY, textPaint)
        val nisStr = if (nisPassed) "PASS" else "FAIL"
        canvas.drawText("Trust: ${(trustScore * 100).toInt()}% | NIS: $nisStr", 30.0f, hudTextY + 40.0f, textPaint)
        canvas.drawText("95% Uncertainty: ±${String.format("%.1f", ellipse.semiMajorAxisM)} m", 30.0f, hudTextY + 80.0f, textPaint)
        canvas.drawText("Semi-Axes: [a=${String.format("%.2f", ellipse.semiMajorAxisM)}m, b=${String.format("%.2f", ellipse.semiMinorAxisM)}m]", 30.0f, hudTextY + 120.0f, textPaint)
    }
}
