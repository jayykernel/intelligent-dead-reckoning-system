package com.example.idr.map_matching

import kotlin.math.*

/**
 * Road Segment definition for map matching.
 */
data class RoadSegment(
    val segmentId: Int,
    val osmWayId: Long,
    val pStart: DoubleArray,
    val pEnd: DoubleArray,
    val highwayType: String = "primary",
    val oneway: Boolean = false
) {
    val vec = doubleArrayOf(pEnd[0] - pStart[0], pEnd[1] - pStart[1])
    val length = hypot(vec[0], vec[1])
    val dir = if (length > 1e-6) doubleArrayOf(vec[0] / length, vec[1] / length) else doubleArrayOf(0.0, 0.0)
    val bearingDeg = if (length > 1e-6) {
        val deg = Math.toDegrees(atan2(dir[0], dir[1]))
        ((deg % 360.0) + 360.0) % 360.0
    } else {
        0.0
    }
    val midpoint = doubleArrayOf((pStart[0] + pEnd[0]) * 0.5, (pStart[1] + pEnd[1]) * 0.5)

    fun projectPoint(pointEnu: DoubleArray): Triple<DoubleArray, Double, Double> {
        val pt = doubleArrayOf(pointEnu[0], pointEnu[1])
        if (length < 1e-6) {
            return Triple(pStart, hypot(pt[0] - pStart[0], pt[1] - pStart[1]), 0.0)
        }

        val v = doubleArrayOf(pt[0] - pStart[0], pt[1] - pStart[1])
        val t = (v[0] * dir[0] + v[1] * dir[1]) / length
        val tClamped = max(0.0, min(1.0, t))

        val proj = doubleArrayOf(pStart[0] + tClamped * vec[0], pStart[1] + tClamped * vec[1])
        val perpDist = hypot(pt[0] - proj[0], pt[1] - proj[1])

        return Triple(proj, perpDist, tClamped)
    }
}

/**
 * Candidate road segment match.
 */
data class CandidateSegment(
    val seg: RoadSegment,
    val proj: DoubleArray,
    val dist: Double
)

/**
 * Lightweight Road Network container.
 */
class RoadNetwork(
    val lat0: Double,
    val lon0: Double,
    val alt0: Double = 0.0
) {
    val segments = mutableListOf<RoadSegment>()

    fun findCandidateSegments(
        pointEnu: DoubleArray,
        radiusM: Double = 30.0,
        maxCandidates: Int = 10
    ): List<CandidateSegment> {
        if (segments.isEmpty()) return emptyList()

        val pt = doubleArrayOf(pointEnu[0], pointEnu[1])
        val queryRadius = radiusM + 50.0 // Max expected half-segment length

        val candidates = mutableListOf<CandidateSegment>()
        for (seg in segments) {
            val distToMidpoint = hypot(pt[0] - seg.midpoint[0], pt[1] - seg.midpoint[1])
            if (distToMidpoint <= queryRadius) {
                val (proj, dist, _) = seg.projectPoint(pt)
                if (dist <= radiusM) {
                    candidates.add(CandidateSegment(seg, proj, dist))
                }
            }
        }

        candidates.sortBy { it.dist }
        return if (candidates.size > maxCandidates) candidates.subList(0, maxCandidates) else candidates
    }
}

/**
 * Map Matching result structure.
 */
data class MapMatchingResult(
    val rawPos: DoubleArray,
    val snappedPos: DoubleArray,
    val snapped: Boolean,
    val confidence: Double,
    val matchedSegmentId: Int? = null,
    val osmWayId: Long? = null,
    val crossTrackErrorM: Double = 0.0,
    val fallbackReason: String? = null
)

/**
 * Internal evaluated candidate.
 */
data class EvaluatedCandidate(
    val seg: RoadSegment,
    val proj: DoubleArray,
    val dist: Double,
    val emitP: Double
)

/**
 * History step for Viterbi state tracking.
 */
data class HistoryStep(
    val candidates: List<EvaluatedCandidate>,
    val viterbi: Map<Int, Double>,
    val raw: DoubleArray
)

/**
 * HMM-based Map Matcher supporting Car and Two-Wheeler profiles with No-Snap Fallbacks.
 */
class HMMMapMatcher(
    val roadNetwork: RoadNetwork,
    vehicleType: String = "car"
) {
    var vehicleType: String = vehicleType
        private set

    var searchRadius: Double = 25.0
        private set
    var sigmaZ: Double = 5.0
        private set
    var beta: Double = 5.0
        private set
    var headingWeight: Double = 2.0
        private set
    var maxDeviationM: Double = 25.0
        private set
    var minConfidence: Double = 1e-3
        private set

    val historyStates = mutableListOf<HistoryStep>()
    var lastPosEnu: DoubleArray? = null
        private set
    var lastMatchedSeg: RoadSegment? = null
        private set

    init {
        configureProfile(vehicleType)
    }

    fun configureProfile(type: String) {
        vehicleType = type
        if (type == "two_wheeler") {
            searchRadius = 45.0
            sigmaZ = 10.0
            beta = 8.0
            headingWeight = 0.5
            maxDeviationM = 50.0
            minConfidence = 1e-4
        } else {
            searchRadius = 25.0
            sigmaZ = 5.0
            beta = 5.0
            headingWeight = 2.0
            maxDeviationM = 25.0
            minConfidence = 1e-3
        }
    }

    fun setVehicleType(type: String) {
        configureProfile(type)
    }

    fun emissionProb(
        distM: Double,
        headingDeg: Double?,
        seg: RoadSegment
    ): Double {
        val pDist = (1.0 / (sqrt(2.0 * Math.PI) * sigmaZ)) * exp(-0.5 * (distM / sigmaZ).pow(2.0))

        val pHeading = if (headingDeg != null && seg.length > 1.0) {
            var angleDiff = abs((headingDeg - seg.bearingDeg + 180.0) % 360.0 - 180.0)
            if (!seg.oneway) {
                val revDiff = abs((headingDeg - (seg.bearingDeg + 180.0) + 180.0) % 360.0 - 180.0)
                angleDiff = min(angleDiff, revDiff)
            }
            exp(-0.5 * (Math.toRadians(angleDiff) * headingWeight).pow(2.0))
        } else {
            1.0
        }

        return pDist * pHeading
    }

    fun transitionProb(
        prevSeg: RoadSegment,
        currSeg: RoadSegment,
        prevProj: DoubleArray,
        currProj: DoubleArray,
        prevRaw: DoubleArray,
        currRaw: DoubleArray
    ): Double {
        val sameSeg = (prevSeg.segmentId == currSeg.segmentId)
        val dRouteEuclidean = hypot(currProj[0] - prevProj[0], currProj[1] - prevProj[1])

        val dRoute = if (!sameSeg) {
            val pStart1 = prevSeg.pStart
            val pEnd1 = prevSeg.pEnd
            val pStart2 = currSeg.pStart
            val pEnd2 = currSeg.pEnd

            val connected = (hypot(pStart1[0] - pStart2[0], pStart1[1] - pStart2[1]) < 2.0 ||
                    hypot(pEnd1[0] - pStart2[0], pEnd1[1] - pStart2[1]) < 2.0 ||
                    hypot(pStart1[0] - pEnd2[0], pStart1[1] - pEnd2[1]) < 2.0 ||
                    hypot(pEnd1[0] - pEnd2[0], pEnd1[1] - pEnd2[1]) < 2.0)

            if (!connected) {
                dRouteEuclidean + 200.0
            } else {
                dRouteEuclidean
            }
        } else {
            dRouteEuclidean
        }

        val dRaw = hypot(currRaw[0] - prevRaw[0], currRaw[1] - prevRaw[1])
        val deltaD = abs(dRoute - dRaw)

        return (1.0 / beta) * exp(-deltaD / beta)
    }

    fun matchPoint(
        rawPosEnu: DoubleArray,
        headingDeg: Double? = null
    ): MapMatchingResult {
        val pt = doubleArrayOf(rawPosEnu[0], rawPosEnu[1])
        val candidates = roadNetwork.findCandidateSegments(
            pointEnu = rawPosEnu,
            radiusM = searchRadius,
            maxCandidates = 8
        )

        // 1. Fallback: No candidate in radius
        if (candidates.isEmpty()) {
            return MapMatchingResult(
                rawPos = rawPosEnu,
                snappedPos = rawPosEnu,
                snapped = false,
                confidence = 0.0,
                fallbackReason = "NO_CANDIDATE_ROAD_IN_RADIUS"
            )
        }

        // 2. Emission filtering
        val currentCandidates = mutableListOf<EvaluatedCandidate>()
        for (cand in candidates) {
            if (cand.dist > maxDeviationM) continue
            val emitP = emissionProb(cand.dist, headingDeg, cand.seg)
            if (emitP >= minConfidence) {
                currentCandidates.add(
                    EvaluatedCandidate(
                        seg = cand.seg,
                        proj = cand.proj,
                        dist = cand.dist,
                        emitP = emitP
                    )
                )
            }
        }

        // 3. Fallback: Low emission confidence
        if (currentCandidates.isEmpty()) {
            return MapMatchingResult(
                rawPos = rawPosEnu,
                snappedPos = rawPosEnu,
                snapped = false,
                confidence = 0.0,
                fallbackReason = "LOW_EMISSION_CONFIDENCE"
            )
        }

        // 4. First step
        if (historyStates.isEmpty()) {
            val bestCand = currentCandidates.maxByOrNull { it.emitP }!!
            val viterbiMap = currentCandidates.mapIndexed { idx, c ->
                idx to ln(max(1e-12, c.emitP))
            }.toMap()

            historyStates.add(
                HistoryStep(
                    candidates = currentCandidates,
                    viterbi = viterbiMap,
                    raw = pt
                )
            )
            lastPosEnu = rawPosEnu
            lastMatchedSeg = bestCand.seg

            val snapped3d = doubleArrayOf(
                bestCand.proj[0],
                bestCand.proj[1],
                if (rawPosEnu.size > 2) rawPosEnu[2] else 0.0
            )

            return MapMatchingResult(
                rawPos = rawPosEnu,
                snappedPos = snapped3d,
                snapped = true,
                confidence = bestCand.emitP,
                matchedSegmentId = bestCand.seg.segmentId,
                osmWayId = bestCand.seg.osmWayId,
                crossTrackErrorM = bestCand.dist
            )
        }

        // 5. Viterbi transition
        val prevStep = historyStates.last()
        val prevCands = prevStep.candidates
        val prevViterbi = prevStep.viterbi
        val prevRaw = prevStep.raw

        val currViterbi = mutableMapOf<Int, Double>()

        for ((currIdx, cCurr) in currentCandidates.withIndex()) {
            var maxLogProb = Double.NEGATIVE_INFINITY

            for ((prevIdx, cPrev) in prevCands.withIndex()) {
                val transP = transitionProb(
                    cPrev.seg,
                    cCurr.seg,
                    cPrev.proj,
                    cCurr.proj,
                    prevRaw,
                    pt
                )
                val prevLogP = prevViterbi[prevIdx] ?: -1e6
                val logP = prevLogP + ln(max(1e-12, transP)) + ln(max(1e-12, cCurr.emitP))

                if (logP > maxLogProb) {
                    maxLogProb = logP
                }
            }
            currViterbi[currIdx] = maxLogProb
        }

        val bestEntry = currViterbi.maxByOrNull { it.value }!!
        val bestCand = currentCandidates[bestEntry.key]

        // 6. Fallback: Topological discontinuity
        if (bestEntry.value < -40.0) {
            return MapMatchingResult(
                rawPos = rawPosEnu,
                snappedPos = rawPosEnu,
                snapped = false,
                confidence = 0.0,
                fallbackReason = "TOPOLOGICAL_DISCONTINUITY"
            )
        }

        historyStates.add(
            HistoryStep(
                candidates = currentCandidates,
                viterbi = currViterbi,
                raw = pt
            )
        )
        if (historyStates.size > 50) {
            historyStates.removeAt(0)
        }

        lastPosEnu = rawPosEnu
        lastMatchedSeg = bestCand.seg

        val snapped3d = doubleArrayOf(
            bestCand.proj[0],
            bestCand.proj[1],
            if (rawPosEnu.size > 2) rawPosEnu[2] else 0.0
        )

        return MapMatchingResult(
            rawPos = rawPosEnu,
            snappedPos = snapped3d,
            snapped = true,
            confidence = bestCand.emitP,
            matchedSegmentId = bestCand.seg.segmentId,
            osmWayId = bestCand.seg.osmWayId,
            crossTrackErrorM = bestCand.dist
        )
    }

    fun matchTrajectory(
        positionsEnu: List<DoubleArray>,
        headingsDeg: List<Double?>? = null
    ): List<MapMatchingResult> {
        historyStates.clear()
        val results = mutableListOf<MapMatchingResult>()
        for (i in positionsEnu.indices) {
            val h = headingsDeg?.getOrNull(i)
            results.add(matchPoint(positionsEnu[i], headingDeg = h))
        }
        return results
    }
}
