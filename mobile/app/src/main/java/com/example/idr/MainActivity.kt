package com.example.idr

import android.os.Bundle
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import com.example.idr.ui.ConfidenceEllipseView

/**
 * Main navigation activity with live Confidence Ellipse UI rendering (N4).
 */
class MainActivity : AppCompatActivity() {

    private lateinit var ellipseView: ConfidenceEllipseView
    private lateinit var tvFilterStatus: TextView
    private lateinit var tvCovarianceInfo: TextView

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        ellipseView = findViewById(R.id.confidenceEllipseView)
        tvFilterStatus = findViewById(R.id.tvFilterStatus)
        tvCovarianceInfo = findViewById(R.id.tvCovarianceInfo)

        // Initialize with steady-state nominal GNSS covariance
        updateNavigationState(
            pEE = 1.0,
            pNN = 1.2,
            pEN = 0.2,
            mode = "GNSS_AIDED",
            trust = 1.0f,
            heading = 0.0f
        )
    }

    /**
     * Updates the UI when a new state estimate is produced by the fusion pipeline.
     */
    fun updateNavigationState(
        pEE: Double,
        pNN: Double,
        pEN: Double,
        mode: String,
        trust: Float,
        heading: Float
    ) {
        ellipseView.updateState(
            pEE = pEE,
            pNN = pNN,
            pEN = pEN,
            currentMode = mode,
            trust = trust,
            heading = heading
        )

        val trace = pEE + pNN
        tvFilterStatus.text = "Mode: $mode | Trust: ${(trust * 100).toInt()}%"
        tvCovarianceInfo.text = String.format("Pos Var Trace: %.2f m² | P_EE=%.2f, P_NN=%.2f", trace, pEE, pNN)
    }
}
