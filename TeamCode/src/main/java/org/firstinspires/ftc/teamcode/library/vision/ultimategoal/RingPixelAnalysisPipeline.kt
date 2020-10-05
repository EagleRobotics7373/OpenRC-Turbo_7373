package org.firstinspires.ftc.teamcode.library.vision.ultimategoal

import org.firstinspires.ftc.teamcode.library.functions.truncate
import org.firstinspires.ftc.teamcode.library.vision.base.ImageResolution
import org.firstinspires.ftc.teamcode.library.vision.base.ResolutionPipeline

import org.opencv.core.*
import org.opencv.imgproc.Imgproc.*
import kotlin.math.pow

class RingPixelAnalysisPipeline() : ResolutionPipeline() {

    // Public variable allowing OpModes to access number of rings
    var numberOfRings : Int? = null
        private set

    // The mat that we will analyze, alone with the channel number we need to extract from HSV
    private var rgbMat: Mat = Mat()
    private var fullHsvMat: Mat = Mat()
    private var hsvMat: Mat = Mat()

    override var resolution : ImageResolution = ImageResolution.R_720x480

    override fun processFrame(input: Mat): Mat {
        if (tracking) {

            rgbMat = input
            extractSingleChannel()

            val statsArray = trackRings()

            numberOfRings = when {
                statsArray[0].value < 170 -> 0
                statsArray[1].value < 170 -> 1
                else -> 4
            }

            addLabels(mat = hsvMat, ringStats = statsArray)
            if (!shouldKeepTracking) tracking = false
            return hsvMat
        }
        else {
            return input
        }
    }

    private fun extractSingleChannel() {
        cvtColor(rgbMat, fullHsvMat, COLOR_RGB2HLS)
        Core.extractChannel(fullHsvMat, hsvMat, 2)
    }

    private fun trackRings()
            : List<RingStats> {

        // Get the maximum number of rows and columns which we will use in the next step
        val maxRows = hsvMat.rows()
        val maxCols = hsvMat.cols()

        val x0                  = UltimateGoalVisionConstants.BOTTOM_RING_CENTER_X
                                    .times(resolution.scale).toInt()
                                    .coerceIn(0 until maxCols)
        val y0                  = UltimateGoalVisionConstants.BOTTOM_RING_CENTER_Y
                                    .times(resolution.scale).toInt()
                                    .coerceIn(0 until maxRows)
        val width               = UltimateGoalVisionConstants.RING_WIDTH
                                    .times(resolution.scale).toInt()
        val height              = UltimateGoalVisionConstants.RING_HEIGHT
                                    .times(resolution.scale).toInt()
        val spacing             = UltimateGoalVisionConstants.RING_SPACING
                                    .times(resolution.scale).toInt()
        val searchSkipLength    = UltimateGoalVisionConstants.SEARCH_SKIP
        val searchType          = UltimateGoalVisionConstants.SEARCH_TYPE


        // Get the distance from the center point to the edge
        val xDistanceToEdge = (width / 2)
        val yDistanceToEdge = (height / 2)


        return (0 until 2).map {
            // Record the start time so we can see how long this takes
            val startingRuntime = System.currentTimeMillis()

            // Get the actual y value of the ring based on its number
            val y = y0 - (height * (it + 1)) - spacing * it


            // Find the bounding edges of the search zone and keep it within the image bounds
            val xMin = (x0 - xDistanceToEdge).coerceIn(0 until maxCols)
            val xMax = (x0 + xDistanceToEdge).coerceIn(0 until maxCols)
            val yMin = (y - yDistanceToEdge).coerceIn(0 until maxRows)
            val yMax = (y + yDistanceToEdge).coerceIn(0 until maxRows)


            val points = emptyList<Double>().toMutableList()

            // Calculate the average value of the points in the bounding box
            var pointsSum = 0.0
            (yMin until yMax step searchSkipLength).forEach { rowInc ->
                (xMin until xMax step searchSkipLength).forEach { colInc ->
                    val pointValue = hsvMat.get(rowInc, colInc)[0]
                    hsvMat.put(rowInc, colInc, if (it == 0) 255.0 else 0.0)
                    pointsSum += pointValue
                    points.add(pointValue)
                }
            }
            val average = (pointsSum / points.count())

            // By making the standard deviation calculation conditional, we save CPU
            val finalValue = when (searchType) {
                SearchType.STDEV -> {
                    // Calculate the standard deviation of the points in the bounding box
                    var stdDevSum = 0.0
                    points.forEach { point -> stdDevSum += (point - average).pow(2.0) }
                    (stdDevSum / points.count() - 1).pow(0.5)
                }
                else -> average
            }

            // Annotate on the screen to indicate the bounding box
            val centerPointInverse = inverseColorAtPoint(hsvMat, Point(x0.toDouble(), y.toDouble()))
            rectangle(
                    hsvMat,
                    Point(xMin.toDouble(), yMin.toDouble()),
                    Point(xMax.toDouble(), yMax.toDouble()),
                    Scalar(centerPointInverse),
                    (5 * resolution.scale).toInt()
            )
            circle(
                    hsvMat,
                    Point(x0.toDouble(), y.toDouble()),
                    (7 * resolution.scale).toInt(),
                    Scalar(centerPointInverse),
                    (8 * resolution.scale).toInt()
            )

            // Find the end time for scanning this ring
            val elapsedRuntime = System.currentTimeMillis() - startingRuntime

            // Return stats for the ring
            return@map RingStats(
                    it,
                    finalValue.toInt(), searchType, points.count(),
                    elapsedRuntime
            )

        }
    }

    private fun inverseColorAtPoint(mat: Mat, point: Point): DoubleArray {
        val newPoint = point.coerceIn(mat)
        return mat.get(newPoint.y.toInt(), newPoint.x.toInt()) // Get color at this point
                .map { 255 - it }                              // For each channel, invert the color
//                .dropLast(1)
                .toDoubleArray()                               // Re-convert to DoubleArray
    }


    private fun addLabels(mat: Mat, ringStats: List<RingStats>? = null) {

        // Place text representing the team name (at the top)
        val teamNameStartPoint = Point(5.0, 30.0)
                .times(resolution.scale)
                .coerceIn(mat)
        putText(mat, "Eagle Robotics Team 7373", teamNameStartPoint, 
                FONT_HERSHEY_SIMPLEX, 1 * resolution.scale, 
                Scalar(inverseColorAtPoint(mat, teamNameStartPoint)), 2)

        // Place text indicating the detector purpose (below the team name)
        val detectorNameStartPoint = Point(5.0, 50.0)
                .times(resolution.scale)
                .coerceIn(mat)
        putText(mat, "ULTIMATE GOAL CV Ring Detection Pipeline", detectorNameStartPoint,
                FONT_HERSHEY_SIMPLEX, 0.55 * resolution.scale, 
                Scalar(inverseColorAtPoint(mat, detectorNameStartPoint)), 2)

        // For each ring, show the RingStats found during image processing
        ringStats?.forEach {
            val ringTextStartPoint = Point(
                    5.0, mat.rows() - (30.0 * resolution.scale * (it.ringNumber + 1)))
                    .coerceIn(mat)
            print("Ring ${it.ringNumber} START POINT = $ringTextStartPoint with rows ${mat.rows()}")
            putText(mat,
                    "Ring #${it.ringNumber}: scanned ${it.valueType} as ${it.value} " +
                            "with ${it.samples} samples for ${it.runtimeAsSeconds.truncate(4)} seconds",
                    ringTextStartPoint,
                    FONT_HERSHEY_SIMPLEX, 0.5 * resolution.scale,
                    Scalar(inverseColorAtPoint(mat, ringTextStartPoint)), 2)
        }

        // If we are being sent a ring result, then show that result
        val resultTextStartPoint = Point(5.0, mat.rows()-10.0)
                .times(resolution.scale)
                .coerceIn(mat)
        val resultText = when {
            numberOfRings == null   -> "No detection stored."
            ringStats == null       -> "Last detected $numberOfRings rings."
            else                    -> "Detecting $numberOfRings rings."
        }
        putText(mat, resultText,
                resultTextStartPoint,
                FONT_HERSHEY_SIMPLEX, 0.6 * resolution.scale,
                Scalar(inverseColorAtPoint(mat, resultTextStartPoint)), 2)
    }

    class RingStats(
            val ringNumber: Int,
            val value     : Int,
            val valueType : SearchType,
            val samples   : Int,
            val runtime   : Long
    ) {
        val runtimeAsSeconds: Double get() = (runtime / 1000.0)
    }


    enum class SearchType {
        AVERAGE, STDEV
    }

    fun Point.times(increment: Double): Point {
        return Point(this.x.times(increment), this.y.times(increment))
    }

    fun Point.coerceIn(mat: Mat): Point {
        return Point(this.x.coerceIn(0.0, mat.cols()-1.0), this.y.coerceIn(0.0, mat.rows()-1.0))
    }

}