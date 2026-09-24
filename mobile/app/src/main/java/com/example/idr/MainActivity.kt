package com.example.idr

import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.widget.Button
import android.widget.ProgressBar
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import com.example.idr.fusion.FusionEngine
import com.example.idr.ui.ConfidenceEllipseView
import org.osmdroid.config.Configuration
import org.osmdroid.tileprovider.tilesource.ITileSource
import org.osmdroid.tileprovider.tilesource.TileSourceFactory
import org.osmdroid.tileprovider.tilesource.XYTileSource
import org.osmdroid.util.GeoPoint
import org.osmdroid.views.MapView
import org.osmdroid.views.overlay.Polyline
import kotlin.math.*

import java.io.File

class MainActivity : AppCompatActivity() {

    companion object {
        // High-performance OpenStreetMap raster basemap powered by CARTO CDN (Compliant with OSM usage policy)
        val OSM_VOYAGER: ITileSource = XYTileSource(
            "CartoVoyager",
            0, 20, 256, ".png",
            arrayOf(
                "https://a.basemaps.cartocdn.com/rastertiles/voyager/",
                "https://b.basemaps.cartocdn.com/rastertiles/voyager/",
                "https://c.basemaps.cartocdn.com/rastertiles/voyager/",
                "https://d.basemaps.cartocdn.com/rastertiles/voyager/"
            ),
            "© OpenStreetMap contributors, © CARTO"
        )
    }

    private lateinit var mapView: MapView
    private lateinit var ellipseView: ConfidenceEllipseView
    private lateinit var tvModeBadge: TextView
    private lateinit var tvTrustLabel: TextView
    private lateinit var pbTrustScore: ProgressBar
    private lateinit var tvNisStatus: TextView
    private lateinit var tvUncertainty: TextView
    private lateinit var tvSpeedHeading: TextView
    private lateinit var btnToggleOutage: Button

    private val fusionEngine = FusionEngine(dt = 0.1, enableAi = true)
    private val mainHandler = Handler(Looper.getMainLooper())
    private var isPlaying = true
    private var isOutageSimulated = false
    private var simTime = 0.0

    // Arbitrary origin for ENU conversion
    private val ORIGIN_LAT = 12.9716 // Bangalore center
    private val ORIGIN_LON = 77.5946
    private val trajectoryTrail = Polyline()

    private val playbackRunnable = object : Runnable {
        override fun run() {
            if (!isPlaying) return
            simTime += 0.1

            // 1. Simulate Input Data
            val accRaw = doubleArrayOf(0.0, 0.0, 9.81)
            val gyroRaw = doubleArrayOf(0.0, 0.0, 0.0)

            // Linear movement
            val pos = doubleArrayOf(10.0 * simTime, 0.0, 0.0)
            val vel = doubleArrayOf(10.0, 0.0, 0.0)

            // Toggle outage based on user interaction
            val isGnssAvailable = !isOutageSimulated

            // 2. Fusion Step
            val result = fusionEngine.step(
                accRaw, gyroRaw,
                if (isGnssAvailable) pos else null,
                if (isGnssAvailable) vel else null,
                isGnssAvailable, simTime,
                gnssAccM = 2.0, gnssSatCount = 12, gnssAvgCn0 = 35.0
            )

            // 3. Update UI
            val p = result["pos"] as DoubleArray
            val mode = result["mode"] as String
            val trust = (result["trust_score"] as Double).toFloat()
            val cov2d = result["cov_2d"] as DoubleArray
            val heading = (result["heading"] as Double).toFloat()
            val nisPassed = result["gnss_passed"] as Boolean

            updateMap(p, heading)

            ellipseView.updateState(cov2d[0], cov2d[1], cov2d[2], mode, trust, heading, nisPassed)

            // HUD Updates
            tvModeBadge.text = mode
            tvModeBadge.setBackgroundColor(if (mode == "GNSS_AIDED") -16737536 else -65536) // Green/Red
            pbTrustScore.progress = (trust * 100).toInt()
            tvTrustLabel.text = "Trust: ${(trust * 100).toInt()}%"
            tvNisStatus.text = "NIS: ${if (nisPassed) "PASS" else "FAIL"}"
            tvUncertainty.text = "Uncertainty (95%): ±${String.format("%.1f", sqrt(cov2d[0] + cov2d[1]))} m"
            tvSpeedHeading.text = "Yaw: ${heading.toInt()}°"

            mainHandler.postDelayed(this, 100)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val config = Configuration.getInstance()
        // 1. Load preferences FIRST
        config.load(applicationContext, getPreferences(MODE_PRIVATE))

        // 2. Comply with API 30+ Scoped Storage by setting explicitly to App Cache
        val osmBaseDir = File(cacheDir, "osmdroid")
        osmBaseDir.mkdirs()
        config.osmdroidBasePath = osmBaseDir
        config.osmdroidTileCache = File(osmBaseDir, "tiles")

        // 3. Comply with OpenStreetMap Tile Usage Policy (osm.wiki/Blocked)
        // MUST set unique user-agent after `load` overwrites it, and THEN save it so it works on next boot too.
        config.userAgentValue = "IntelligentDeadReckoningApp/1.0 (+https://github.com/jayanithyan; contact: jayanithyan@example.com)"
        config.save(applicationContext, getPreferences(MODE_PRIVATE))

        setContentView(R.layout.activity_main)

        mapView = findViewById(R.id.mapView)
        // Set compliant map tile source
        mapView.setTileSource(OSM_VOYAGER)
        mapView.setMultiTouchControls(true)
        mapView.controller.setZoom(16.0)
        mapView.overlays.add(trajectoryTrail)

        ellipseView = findViewById(R.id.confidenceEllipseView)
        tvModeBadge = findViewById(R.id.tvModeBadge)
        tvTrustLabel = findViewById(R.id.tvTrustLabel)
        pbTrustScore = findViewById(R.id.pbTrustScore)
        tvNisStatus = findViewById(R.id.tvNisStatus)
        tvUncertainty = findViewById(R.id.tvUncertainty)
        tvSpeedHeading = findViewById(R.id.tvSpeedHeading)
        btnToggleOutage = findViewById(R.id.btnToggleOutage)

        btnToggleOutage.setOnClickListener {
            isOutageSimulated = !isOutageSimulated
            btnToggleOutage.text = if (isOutageSimulated) "Restore GNSS" else "Simulate 60s Outage"
            btnToggleOutage.setBackgroundColor(if (isOutageSimulated) -16711936 else -4568010) // Green/Red
        }

        mainHandler.post(playbackRunnable)
    }

    override fun onResume() {
        super.onResume()
        mapView.onResume()
    }

    override fun onPause() {
        super.onPause()
        mapView.onPause()
    }

    override fun onDestroy() {
        super.onDestroy()
        mainHandler.removeCallbacks(playbackRunnable)
        mapView.onDetach()
    }

    private fun updateMap(p: DoubleArray, heading: Float) {
        // Simple ENU to LatLng (Approximation)
        val lat = ORIGIN_LAT + (p[1] / 111320.0)
        val lon = ORIGIN_LON + (p[0] / (111320.0 * cos(Deg2Rad(ORIGIN_LAT))))
        val geoPoint = GeoPoint(lat, lon)

        mapView.controller.setCenter(geoPoint)
        trajectoryTrail.addPoint(geoPoint)
        mapView.invalidate()
    }

    private fun Deg2Rad(deg: Double) = deg * PI / 180.0
}
