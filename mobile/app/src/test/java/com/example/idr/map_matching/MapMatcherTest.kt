package com.example.idr.map_matching

import org.json.JSONArray
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.File

class MapMatcherTest {

    @Test
    fun testMapMatcherNumericalParity() {
        val networkPath = "../../engine/map_matching/tests/vectors/map_matcher_network.json"
        val inPath = "../../engine/map_matching/tests/vectors/map_matcher_in.json"
        val outPath = "../../engine/map_matching/tests/vectors/map_matcher_out.json"

        val networkFile = File(networkPath)
        val inFile = File(inPath)
        val outFile = File(outPath)

        assertTrue("Network file not found: ${networkFile.absolutePath}", networkFile.exists())
        assertTrue("In file not found: ${inFile.absolutePath}", inFile.exists())
        assertTrue("Out file not found: ${outFile.absolutePath}", outFile.exists())

        val networkJson = JSONArray(networkFile.readText())
        val inJson = JSONArray(inFile.readText())
        val outJson = JSONArray(outFile.readText())

        val roadNetwork = RoadNetwork(lat0 = 52.4, lon0 = -1.5)
        for (i in 0 until networkJson.length()) {
            val segObj = networkJson.getJSONObject(i)
            val pStartArr = segObj.getJSONArray("p_start")
            val pEndArr = segObj.getJSONArray("p_end")

            val seg = RoadSegment(
                segmentId = segObj.getInt("segment_id"),
                osmWayId = segObj.getLong("osm_way_id"),
                pStart = doubleArrayOf(pStartArr.getDouble(0), pStartArr.getDouble(1)),
                pEnd = doubleArrayOf(pEndArr.getDouble(0), pEndArr.getDouble(1)),
                highwayType = segObj.getString("highway_type"),
                oneway = segObj.getBoolean("oneway")
            )
            roadNetwork.segments.add(seg)
        }

        // Test Car Sequence
        val matcherCar = HMMMapMatcher(roadNetwork, vehicleType = "car")
        for (i in 0 until 6) {
            val inp = inJson.getJSONObject(i)
            val posArr = inp.getJSONArray("pos")
            val pos = doubleArrayOf(posArr.getDouble(0), posArr.getDouble(1), posArr.getDouble(2))
            val heading = inp.getDouble("heading")

            val res = matcherCar.matchPoint(pos, headingDeg = heading)
            val exp = outJson.getJSONObject(i)

            val expSnapped = exp.getBoolean("snapped")
            assertEquals("Snapped flag mismatch at car step $i (${inp.getString("desc")})", expSnapped, res.snapped)

            val expSnappedPos = exp.getJSONArray("snapped_pos")
            for (dim in 0..2) {
                assertEquals("SnappedPos[$dim] mismatch at car step $i", expSnappedPos.getDouble(dim), res.snappedPos[dim], 1e-5)
            }

            assertEquals("Confidence mismatch at car step $i", exp.getDouble("confidence"), res.confidence, 1e-5)
            assertEquals("CrossTrackError mismatch at car step $i", exp.getDouble("cross_track_error_m"), res.crossTrackErrorM, 1e-5)

            if (exp.isNull("matched_segment_id")) {
                assertEquals(null, res.matchedSegmentId)
            } else {
                assertEquals(exp.getInt("matched_segment_id"), res.matchedSegmentId)
            }

            if (exp.isNull("fallback_reason")) {
                assertEquals(null, res.fallbackReason)
            } else {
                assertEquals(exp.getString("fallback_reason"), res.fallbackReason)
            }
        }

        // Test Two-Wheeler Sequence
        val matcherTw = HMMMapMatcher(roadNetwork, vehicleType = "two_wheeler")
        for (i in 6 until inJson.length()) {
            val inp = inJson.getJSONObject(i)
            val posArr = inp.getJSONArray("pos")
            val pos = doubleArrayOf(posArr.getDouble(0), posArr.getDouble(1), posArr.getDouble(2))
            val heading = inp.getDouble("heading")

            val res = matcherTw.matchPoint(pos, headingDeg = heading)
            val exp = outJson.getJSONObject(i)

            val expSnapped = exp.getBoolean("snapped")
            assertEquals("Snapped flag mismatch at TW step $i (${inp.getString("desc")})", expSnapped, res.snapped)

            val expSnappedPos = exp.getJSONArray("snapped_pos")
            for (dim in 0..2) {
                assertEquals("SnappedPos[$dim] mismatch at TW step $i", expSnappedPos.getDouble(dim), res.snappedPos[dim], 1e-5)
            }

            assertEquals("Confidence mismatch at TW step $i", exp.getDouble("confidence"), res.confidence, 1e-5)
            assertEquals("CrossTrackError mismatch at TW step $i", exp.getDouble("cross_track_error_m"), res.crossTrackErrorM, 1e-5)

            if (exp.isNull("matched_segment_id")) {
                assertEquals(null, res.matchedSegmentId)
            } else {
                assertEquals(exp.getInt("matched_segment_id"), res.matchedSegmentId)
            }

            if (exp.isNull("fallback_reason")) {
                assertEquals(null, res.fallbackReason)
            } else {
                assertEquals(exp.getString("fallback_reason"), res.fallbackReason)
            }
        }
    }
}
