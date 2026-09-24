package com.example.idr

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.widget.Button
import android.widget.ProgressBar
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.example.idr.fusion.FusionEngine
import com.example.idr.ui.ConfidenceEllipseView
import org.osmdroid.config.Configuration
import org.osmdroid.tileprovider.tilesource.ITileSource
import org.osmdroid.tileprovider.tilesource.XYTileSource
import org.osmdroid.util.GeoPoint
import org.osmdroid.views.MapView
import org.osmdroid.views.overlay.Polyline
import java.io.File
import kotlin.math.*

/**
 * Intelligent Dead Reckoning - Android Real-Time Navigation Controller
 *
 * Supports two operational modes:
 * 1. LIVE_SENSORS (Default): Reads real device hardware GPS (LocationManager) and IMU (SensorManager)
 *    to track your actual real-world physical location with Dead Reckoning fallback when indoors/outage.
 * 2. DEMO_SIMULATION: Runs a synthetic urban driving loop for demo/benchmarking without requiring physical travel.
 */
class MainActivity : AppCompatActivity(), SensorEventListener, LocationListener {

    companion object {
        private const val PERMISSION_REQUEST_LOCATION = 1001

        val OSM_GLOBAL: ITileSource = XYTileSource(
            "OSM_France",
            0, 20, 256, ".png",
            arrayOf(
                "https://a.tile.openstreetmap.fr/osmfr/",
                "https://b.tile.openstreetmap.fr/osmfr/",
                "https://c.tile.openstreetmap.fr/osmfr/"
            ),
            "© OpenStreetMap contributors"
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
    private lateinit var tvOutageTimer: TextView
    private lateinit var btnModeSwitch: Button
    private lateinit var btnToggleOutage: Button
    private lateinit var btnRecenter: Button

    private val fusionEngine = FusionEngine(dt = 0.1, enableAi = true)
    private val mainHandler = Handler(Looper.getMainLooper())

    // Hardware Managers
    private lateinit var sensorManager: SensorManager
    private lateinit var locationManager: LocationManager
    private var accelSensor: Sensor? = null
    private var gyroSensor: Sensor? = null

    // Operating Mode
    private var isLiveSensorMode = true // true = Real Phone GPS/IMU, false = Demo Loop
    private var isOutageSimulated = false
    private var simTime = 0.0
    private var outageElapsed = 0.0

    // Real-world Geodetic Reference Frame
    private var isOriginSet = false
    private var originLat = 0.0
    private var originLon = 0.0
    private var originAlt = 0.0

    // Latest Live Raw Sensor Readings
    private val latestAccRaw = doubleArrayOf(0.0, 0.0, 9.81)
    private val latestGyroRaw = doubleArrayOf(0.0, 0.0, 0.0)
    private var latestGnssPosEnu: DoubleArray? = null
    private var latestGnssVelEnu: DoubleArray? = null
    private var hasNewGnssMeasurement = false
    private var latestGnssAccM = 10.0
    private var latestGnssSatCount = 0
    private var latestGnssAvgCn0 = 0.0
    private var isGpsFixAcquired = false

    // Trajectory Polyline
    private val trajectoryTrail = Polyline().apply {
        outlinePaint.color = -16737536 // Bright Green Trail
        outlinePaint.strokeWidth = 6.0f
    }

    // Demo Simulation State
    private var currentSimX = 0.0
    private var currentSimY = 0.0
    private var simSpeed = 12.0 // ~43 km/h
    private var simYawRad = 0.0

    // 10 Hz Real-Time EKF Fusion Loop
    private val fusionLoopRunnable = object : Runnable {
        override fun run() {
            simTime += 0.1

            if (isLiveSensorMode) {
                // ==========================================
                // 1. LIVE DEVICE HARDWARE SENSORS PIPELINE
                // ==========================================
                val isGnssAvailable = isGpsFixAcquired && !isOutageSimulated

                if (isOutageSimulated) {
                    outageElapsed += 0.1
                } else {
                    outageElapsed = 0.0
                }

                val gnssPos = if (isGnssAvailable && hasNewGnssMeasurement) latestGnssPosEnu else null
                val gnssVel = if (isGnssAvailable && hasNewGnssMeasurement) latestGnssVelEnu else null
                hasNewGnssMeasurement = false

                // Step ES-EKF with real Accelerometer, Gyroscope and GPS
                val result = fusionEngine.step(
                    accRaw = latestAccRaw.copyOf(),
                    gyroRaw = latestGyroRaw.copyOf(),
                    gnssPosEnu = gnssPos,
                    gnssVelEnu = gnssVel,
                    isGnssAvailable = isGnssAvailable,
                    timestamp = simTime,
                    gnssAccM = latestGnssAccM,
                    gnssSatCount = latestGnssSatCount,
                    gnssAvgCn0 = latestGnssAvgCn0
                )

                renderFusionOutput(result)

            } else {
                // ==========================================
                // 2. DEMO SIMULATION LOOP PIPELINE
                // ==========================================
                runDemoSimulationStep()
            }

            mainHandler.postDelayed(this, 100) // 10 Hz
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // Initialize OSM Configuration
        val config = Configuration.getInstance()
        config.load(applicationContext, getPreferences(MODE_PRIVATE))
        val osmBaseDir = File(cacheDir, "osmdroid")
        osmBaseDir.mkdirs()
        config.osmdroidBasePath = osmBaseDir
        config.osmdroidTileCache = File(osmBaseDir, "tiles")
        config.userAgentValue = "IntelligentDeadReckoningApp/1.0 (+https://github.com/jayanithyan; contact: jayanithyan@example.com)"
        config.save(applicationContext, getPreferences(MODE_PRIVATE))

        setContentView(R.layout.activity_main)

        // Views
        mapView = findViewById(R.id.mapView)
        mapView.setTileSource(OSM_GLOBAL)
        mapView.setMultiTouchControls(true)
        mapView.controller.setZoom(18.0)
        mapView.overlays.add(trajectoryTrail)

        ellipseView = findViewById(R.id.confidenceEllipseView)
        tvModeBadge = findViewById(R.id.tvModeBadge)
        tvTrustLabel = findViewById(R.id.tvTrustLabel)
        pbTrustScore = findViewById(R.id.pbTrustScore)
        tvNisStatus = findViewById(R.id.tvNisStatus)
        tvUncertainty = findViewById(R.id.tvUncertainty)
        tvSpeedHeading = findViewById(R.id.tvSpeedHeading)
        tvOutageTimer = findViewById(R.id.tvOutageTimer)
        btnModeSwitch = findViewById(R.id.btnModeSwitch)
        btnToggleOutage = findViewById(R.id.btnToggleOutage)
        btnRecenter = findViewById(R.id.btnRecenter)

        // Initialize Hardware Sensors
        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        locationManager = getSystemService(Context.LOCATION_SERVICE) as LocationManager
        accelSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        gyroSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

        // Request Location Permissions
        checkAndRequestPermissions()

        // Setup Buttons
        btnModeSwitch.setOnClickListener {
            isLiveSensorMode = !isLiveSensorMode
            if (isLiveSensorMode) {
                btnModeSwitch.text = "Mode: Live GPS"
                btnModeSwitch.setBackgroundColor(-15099694) // Blue
                Toast.makeText(this, "Switched to Real Phone GPS & IMU Mode", Toast.LENGTH_SHORT).show()
            } else {
                btnModeSwitch.text = "Mode: Demo Loop"
                btnModeSwitch.setBackgroundColor(-10011977) // Purple
                Toast.makeText(this, "Switched to Demo Simulation Mode", Toast.LENGTH_SHORT).show()
                // If origin not set, use Bangalore center for demo
                if (!isOriginSet) {
                    originLat = 12.9716
                    originLon = 77.5946
                    isOriginSet = true
                    mapView.controller.setCenter(GeoPoint(originLat, originLon))
                }
            }
        }

        btnToggleOutage.setOnClickListener {
            isOutageSimulated = !isOutageSimulated
            if (isOutageSimulated) {
                btnToggleOutage.text = "Restore GNSS"
                btnToggleOutage.setBackgroundColor(-16737536) // Green to restore
            } else {
                btnToggleOutage.text = "Simulate Outage"
                btnToggleOutage.setBackgroundColor(-4568010) // Red to simulate
            }
        }

        btnRecenter.setOnClickListener {
            if (isOriginSet) {
                val currentPos = fusionEngine.ekf.p
                val lat = originLat + (currentPos[1] / 111320.0)
                val lon = originLon + (currentPos[0] / (111320.0 * cos(Deg2Rad(originLat))))
                mapView.controller.animateTo(GeoPoint(lat, lon), 18.0, 500L)
            }
        }

        mainHandler.post(fusionLoopRunnable)
    }

    private fun checkAndRequestPermissions() {
        val fineLocationGranted = ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
        val coarseLocationGranted = ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED

        if (!fineLocationGranted || !coarseLocationGranted) {
            ActivityCompat.requestPermissions(
                this,
                arrayOf(Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION),
                PERMISSION_REQUEST_LOCATION
            )
        } else {
            startLocationUpdates()
        }
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == PERMISSION_REQUEST_LOCATION && grantResults.isNotEmpty() && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            startLocationUpdates()
        }
    }

    private fun startLocationUpdates() {
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED &&
            ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            return
        }

        // Request GPS Updates from real phone hardware
        try {
            if (locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER)) {
                locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 500L, 0.1f, this)
            }
            if (locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER)) {
                locationManager.requestLocationUpdates(LocationManager.NETWORK_PROVIDER, 1000L, 0.5f, this)
            }
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }

    override fun onResume() {
        super.onResume()
        mapView.onResume()
        accelSensor?.let { sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
        gyroSensor?.let { sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
    }

    override fun onPause() {
        super.onPause()
        mapView.onPause()
        sensorManager.unregisterListener(this)
    }

    override fun onDestroy() {
        super.onDestroy()
        mainHandler.removeCallbacks(fusionLoopRunnable)
        try { locationManager.removeUpdates(this) } catch (e: Exception) {}
        mapView.onDetach()
    }

    // ==========================================
    // SENSOR EVENT LISTENER (IMU ACCEL & GYRO)
    // ==========================================
    override fun onSensorChanged(event: SensorEvent?) {
        if (event == null) return
        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                latestAccRaw[0] = event.values[0].toDouble()
                latestAccRaw[1] = event.values[1].toDouble()
                latestAccRaw[2] = event.values[2].toDouble()
            }
            Sensor.TYPE_GYROSCOPE -> {
                latestGyroRaw[0] = event.values[0].toDouble()
                latestGyroRaw[1] = event.values[1].toDouble()
                latestGyroRaw[2] = event.values[2].toDouble()
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    // ==========================================
    // LOCATION LISTENER (REAL PHONE GPS FIX)
    // ==========================================
    override fun onLocationChanged(location: Location) {
        val lat = location.latitude
        val lon = location.longitude
        val alt = location.altitude

        // First GPS fix sets the origin reference frame to the user's real location
        if (!isOriginSet) {
            originLat = lat
            originLon = lon
            originAlt = alt
            isOriginSet = true

            // Center map on user's exact current location
            mapView.controller.setCenter(GeoPoint(originLat, originLon))
            mapView.controller.setZoom(18.0)
            Toast.makeText(this, "GPS Fix Acquired at Current Location!", Toast.LENGTH_SHORT).show()
        }

        // Convert Geodetic (Lat, Lon, Alt) to Local Tangent Plane ENU
        val pE = (lon - originLon) * (111320.0 * cos(Deg2Rad(originLat)))
        val pN = (lat - originLat) * 111320.0
        val pU = alt - originAlt

        latestGnssPosEnu = doubleArrayOf(pE, pN, pU)

        // Compute velocity from GPS speed & bearing
        if (location.hasSpeed() && location.hasBearing()) {
            val speed = location.speed.toDouble()
            val bearingRad = Deg2Rad(location.bearing.toDouble())
            val vE = speed * sin(bearingRad)
            val vN = speed * cos(bearingRad)
            latestGnssVelEnu = doubleArrayOf(vE, vN, 0.0)
        } else {
            latestGnssVelEnu = null
        }

        latestGnssAccM = if (location.hasAccuracy()) location.accuracy.toDouble() else 3.0
        latestGnssSatCount = 14
        latestGnssAvgCn0 = 36.0
        isGpsFixAcquired = true
        hasNewGnssMeasurement = true
    }

    // ==========================================
    // RENDER FUSION HUD & MAP TRAJECTORY
    // ==========================================
    private fun renderFusionOutput(result: Map<String, Any>) {
        val p = result["pos"] as DoubleArray
        val mode = result["mode"] as String
        val trust = (result["trust_score"] as Double).toFloat()
        val cov2d = result["cov_2d"] as DoubleArray
        val heading = (result["heading"] as Double).toFloat()
        val nisPassed = result["gnss_passed"] as Boolean

        if (isOriginSet) {
            updateMap(p)
        }

        // Update Confidence Ellipse overlay
        ellipseView.updateState(cov2d[0], cov2d[1], cov2d[2], mode, trust, heading, nisPassed)

        // Real-Time HUD Status
        tvModeBadge.text = if (mode == "GNSS_AIDED") "GNSS AIDED" else "DEAD RECKONING"
        tvModeBadge.setBackgroundColor(if (mode == "GNSS_AIDED") -16737536 else -4568010)

        val trustPct = (trust * 100).toInt()
        pbTrustScore.progress = trustPct
        tvTrustLabel.text = "Trust: $trustPct%"
        tvNisStatus.text = "NIS: ${if (nisPassed) "PASS" else "FAIL"}"
        tvNisStatus.setTextColor(if (nisPassed) -16737536 else -4568010)

        val uncertainty95 = sqrt(max(0.01, cov2d[0] + cov2d[1])) * 2.4477
        tvUncertainty.text = "Uncertainty (95%): ±${String.format("%.1f", uncertainty95)} m"

        val vel = result["vel"] as DoubleArray
        val speedMps = hypot(vel[0], vel[1])
        tvSpeedHeading.text = "Speed: ${String.format("%.1f", speedMps)} m/s | Yaw: ${(heading.toInt() + 360) % 360}°"

        if (isOutageSimulated) {
            tvOutageTimer.text = "Outage: ${String.format("%.1f", outageElapsed)}s (Simulated Outage)"
            tvOutageTimer.setTextColor(-4568010)
        } else if (!isGpsFixAcquired && isLiveSensorMode) {
            tvOutageTimer.text = "Status: Waiting for GPS fix..."
            tvOutageTimer.setTextColor(-16737536)
        } else {
            tvOutageTimer.text = "Outage: None (100% Live Fix)"
            tvOutageTimer.setTextColor(-8355712)
        }
    }

    private fun runDemoSimulationStep() {
        val cycleT = simTime % 60.0
        val targetYawRad = when {
            cycleT < 15.0 -> 0.0
            cycleT < 18.0 -> (cycleT - 15.0) * (PI / 6.0)
            cycleT < 30.0 -> PI / 2.0
            cycleT < 33.0 -> PI / 2.0 + (cycleT - 30.0) * (PI / 6.0)
            cycleT < 45.0 -> PI
            cycleT < 48.0 -> PI + (cycleT - 45.0) * (PI / 6.0)
            else -> 3.0 * PI / 2.0
        }

        val yawRate = (targetYawRad - simYawRad) / 0.1
        simYawRad = targetYawRad

        val vx = simSpeed * cos(simYawRad)
        val vy = simSpeed * sin(simYawRad)
        currentSimX += vx * 0.1
        currentSimY += vy * 0.1

        val accRaw = doubleArrayOf((Math.random() - 0.5) * 0.1, (Math.random() - 0.5) * 0.1, 9.81)
        val gyroRaw = doubleArrayOf(0.0, 0.0, yawRate)
        val truePosEnu = doubleArrayOf(currentSimX, currentSimY, 0.0)
        val trueVelEnu = doubleArrayOf(vx, vy, 0.0)

        val isGnssAvailable = !isOutageSimulated
        if (isOutageSimulated) outageElapsed += 0.1 else outageElapsed = 0.0

        val result = fusionEngine.step(
            accRaw = accRaw,
            gyroRaw = gyroRaw,
            gnssPosEnu = if (isGnssAvailable) truePosEnu else null,
            gnssVelEnu = if (isGnssAvailable) trueVelEnu else null,
            isGnssAvailable = isGnssAvailable,
            timestamp = simTime,
            gnssAccM = if (isGnssAvailable) 2.0 else 99.0,
            gnssSatCount = if (isGnssAvailable) 14 else 0,
            gnssAvgCn0 = if (isGnssAvailable) 38.0 else 0.0
        )

        renderFusionOutput(result)
    }

    private fun updateMap(p: DoubleArray) {
        val lat = originLat + (p[1] / 111320.0)
        val lon = originLon + (p[0] / (111320.0 * cos(Deg2Rad(originLat))))
        val geoPoint = GeoPoint(lat, lon)

        mapView.controller.setCenter(geoPoint)
        trajectoryTrail.addPoint(geoPoint)
        mapView.invalidate()
    }

    private fun Deg2Rad(deg: Double) = deg * PI / 180.0
}