package com.example.idr.ui

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.DashPathEffect
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import android.util.AttributeSet
import android.view.View
import kotlin.math.cos
import kotlin.math.sin

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

    // Current State Parameters
    private var ellipse: ConfidenceEllipse = ConfidenceEllipse(semiMajorAxisM = 2.0f, semiMinorAxisM = 1.5f, orientationDeg = 0.0f)
    private var mode: String = "GNSS_AIDED"
    private var trustScore: Float = 1.0f
    private var headingDeg: Float = 0.0f // Heading clockwise from North

    // Pixels per meter scale (e.g., 10 pixels = 1 meter for close inspection)
    private var pixelsPerMeter: Float = 8.0f

    // Drawing Paints
    private val ellipseFillPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
        color = Color.argb(45, 0, 180, 80) // Translucent green by default
    }

    private val ellipseStrokePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 3.5f
        color = Color.rgb(0, 180, 80)
    }

    private val oneSigmaStrokePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 2.0f
        pathEffect = DashPathEffect(floatArrayOf(10f, 10f), 0f)
        color = Color.argb(120, 0, 180, 80)
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
        color = Color.argb(40, 150, 150, 150)
    }

    private val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.DKGRAY
        textSize = 32.0f
    }

    private val ovalRect = RectF()
    private val path = Path()

    /**
     * Update the visual state directly from the fusion engine output.
     */
    fun updateState(
        pEE: Double,
        pNN: Double,
        pEN: Double,
        currentMode: String,
        trust: Float,
        heading: Float
    ) {
        this.ellipse = ConfidenceEllipse.fromCovariance(pEE, pNN, pEN)
        this.mode = currentMode
        this.trustScore = trust
        this.headingDeg = heading

        // Dynamically adjust colors based on operational mode and trust
        when (currentMode) {
            "GNSS_AIDED" -> {
                if (trust < 0.6f) {
                    // Pre-outage warning
                    ellipseFillPaint.color = Color.argb(45, 255, 165, 0) // Amber
                    ellipseStrokePaint.color = Color.rgb(255, 140, 0)
                    oneSigmaStrokePaint.color = Color.argb(120, 255, 140, 0)
                } else {
                    // Healthy GNSS
                    ellipseFillPaint.color = Color.argb(45, 46, 125, 50) // Green
                    ellipseStrokePaint.color = Color.rgb(46, 125, 50)
                    oneSigmaStrokePaint.color = Color.argb(120, 46, 125, 50)
                }
            }
            "PURE_DEAD_RECKONING" -> {
                // Pure dead reckoning (expanding uncertainty)
                ellipseFillPaint.color = Color.argb(55, 198, 40, 40) // Red
                ellipseStrokePaint.color = Color.rgb(198, 40, 40)
                oneSigmaStrokePaint.color = Color.argb(130, 198, 40, 40)
            }
        }

        // Auto-scale pixels per meter so the ellipse always fits nicely on screen
        val maxAxisM = ellipse.semiMajorAxisM.coerceAtLeast(1.0f)
        val targetPpm = (width / (maxAxisM * 3.2f)).coerceIn(1.5f, 25.0f)
        this.pixelsPerMeter = targetPpm

        invalidate() // Request redraw
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)

        val cx = width / 2.0f
        val cy = height / 2.0f

        // 1. Draw concentric distance reference rings (5m, 10m, 20m, 50m)
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
        // Convert math orientation (CCW from East) to Canvas rotation (CW from East/X)
        // In Canvas: +X is East (right), +Y is South (down).
        // Since North is -Y, orientation in ENU maps to -orientationDeg in Canvas.
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

        // 4. Draw Vehicle Pointer (triangle centered at cx, cy, rotated by heading)
        canvas.save()
        canvas.translate(cx, cy)
        canvas.rotate(headingDeg) // 0 deg is North (-Y in Canvas)

        path.reset()
        path.moveTo(0f, -22f) // Forward tip
        path.lineTo(-14f, 16f) // Left rear
        path.lineTo(0f, 10f)   // Center indent
        path.lineTo(14f, 16f)  // Right rear
        path.close()

        canvas.drawPath(path, vehiclePaint)
        canvas.drawPath(path, vehicleHeadingPaint)
        canvas.restore()

        // 5. Render HUD Overlay Metrics
        val hudTextY = 50.0f
        canvas.drawText("Mode: $mode", 30.0f, hudTextY, textPaint)
        canvas.drawText("Trust: ${(trustScore * 100).toInt()}%", 30.0f, hudTextY + 40.0f, textPaint)
        canvas.drawText("95% Uncertainty: ±${String.format("%.1f", ellipse.semiMajorAxisM)} m", 30.0f, hudTextY + 80.0f, textPaint)
        canvas.drawText("Semi-Axes: [a=${String.format("%.2f", ellipse.semiMajorAxisM)}m, b=${String.format("%.2f", ellipse.semiMinorAxisM)}m]", 30.0f, hudTextY + 120.0f, textPaint)
    }
}
