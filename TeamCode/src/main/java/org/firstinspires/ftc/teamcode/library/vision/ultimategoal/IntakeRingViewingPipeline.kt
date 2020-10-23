package org.firstinspires.ftc.teamcode.library.vision.ultimategoal

import org.firstinspires.ftc.teamcode.library.vision.base.ImageResolution
import org.firstinspires.ftc.teamcode.library.vision.base.ResolutionPipeline
import org.firstinspires.ftc.teamcode.library.vision.base.coerceIn
import org.firstinspires.ftc.teamcode.library.vision.base.times
import org.firstinspires.ftc.teamcode.library.vision.ultimategoal.UltimateGoalVisionConstants.*
import org.opencv.core.*
import org.opencv.imgproc.Imgproc.*

class IntakeRingViewingPipeline() : ResolutionPipeline() {

    // Public variable allowing OpModes to access number of rings
    var numSuccessfulOutsideIntake : Int = 0
        private set
    var ringVisibleOutsideIntake: Boolean = false
        private set

    var numSuccessfulInsideIntake: Int = 0
        private set
    var ringVisibleInsideIntake: Boolean = false
        private set

    // The mat that we will analyze, alone with the channel number we need to extract from HSV
    private var rgbMat: Mat = Mat()
    private var fullHlsMat: Mat = Mat()

    override var resolution : ImageResolution = ImageResolution.R_720x480

    override fun processFrame(input: Mat): Mat {
        if (tracking) {

            cvtColor(input, fullHlsMat, COLOR_RGB2HLS)

            val x0                  = UltimateGoalVisionConstants.BOTTOM_RING_CENTER_X
                    .times(resolution.scale).toInt()
            val y0                  = UltimateGoalVisionConstants.BOTTOM_RING_CENTER_Y
                    .times(resolution.scale).toInt()
            val width               = UltimateGoalVisionConstants.RING_WIDTH
                    .times(resolution.scale).toInt()
            val height              = UltimateGoalVisionConstants.RING_HEIGHT
                    .times(resolution.scale).toInt()
            val spacing             = UltimateGoalVisionConstants.RING_SPACING
                    .times(resolution.scale).toInt()

            numSuccessfulOutsideIntake = isRingPresent(x0, y0, width, height)
            ringVisibleOutsideIntake = numSuccessfulOutsideIntake > 10

            numSuccessfulInsideIntake = isRingPresent(x0, y0 + spacing, width, height)
            ringVisibleInsideIntake = numSuccessfulInsideIntake > 7

            addLabels(mat = fullHlsMat, numSuccessful = numSuccessfulOutsideIntake, ringPresent = ringVisibleOutsideIntake)
            if (!shouldKeepTracking) tracking = false
            return fullHlsMat
        }
        else {
            return input
        }
    }


    private fun isRingPresent(x: Int, y: Int, width: Int, height: Int)
            : Int {

        // Get the maximum number of rows and columns which we will use in the next step
        val maxRows = fullHlsMat.rows()
        val maxCols = fullHlsMat.cols()

        val x0 = x.coerceIn(0 until maxCols)
        val y0 = y.coerceIn(0 until maxRows)

        val searchSkipLength    = UltimateGoalVisionConstants.SEARCH_SKIP


        // Get the distance from the center point to the edge
        val xDistanceToEdge = (width / 2)
        val yDistanceToEdge = (height / 2)


        // Record the start time so we can see how long this takes
        val startingRuntime = System.currentTimeMillis()

        // Find the bounding edges of the search zone and keep it within the image bounds
        val xMin = (x0 - xDistanceToEdge).coerceIn(0 until maxCols)
        val xMax = (x0 + xDistanceToEdge).coerceIn(0 until maxCols)
        val yMin = (y0 - yDistanceToEdge).coerceIn(0 until maxRows)
        val yMax = (y0 + yDistanceToEdge).coerceIn(0 until maxRows)

        // Calculate the average value of the points in the bounding box
        var numSuccessfulTmp = 0
        (yMin until yMax step searchSkipLength).forEach { rowInc ->
            (xMin until xMax step searchSkipLength).forEach { colInc ->
                val pointValue = fullHlsMat.get(rowInc, colInc)

                if (
                        pointValue[0] in CONTOUR_HUE_LOWER_BOUND..CONTOUR_HUE_UPPER_BOUND
                        && pointValue[1] in CONTOUR_LUM_LOWER_BOUND..CONTOUR_LUM_UPPER_BOUND
                        && pointValue[2] in CONTOUR_SAT_LOWER_BOUND..CONTOUR_SAT_UPPER_BOUND
                )
                    numSuccessfulTmp++
                fullHlsMat.put(rowInc, colInc, 255.0, 255.0, 255.0)
            }
        }

        // Annotate on the screen to indicate the bounding box
        val centerPointInverse = inverseColorAtPoint(fullHlsMat, Point(x0.toDouble(), y0.toDouble()))
        rectangle(
                fullHlsMat,
                Point(xMin.toDouble(), yMin.toDouble()),
                Point(xMax.toDouble(), yMax.toDouble()),
                Scalar(centerPointInverse),
                (5 * resolution.scale).toInt()
        )
        circle(
                fullHlsMat,
                Point(x0.toDouble(), y0.toDouble()),
                (7 * resolution.scale).toInt(),
                Scalar(centerPointInverse),
                (8 * resolution.scale).toInt()
        )

        // Find the end time for scanning this ring
        val elapsedRuntime = System.currentTimeMillis() - startingRuntime

        // Return stats for the ring
        return numSuccessfulTmp
    }

    private fun inverseColorAtPoint(mat: Mat, point: Point): DoubleArray {
        val newPoint = point.coerceIn(mat)
        return mat.get(newPoint.y.toInt(), newPoint.x.toInt()) // Get color at this point
                .map { 255 - it }                              // For each channel, invert the color
//                .dropLast(1)
                .toDoubleArray()                               // Re-convert to DoubleArray
    }


    private fun addLabels(mat: Mat, numSuccessful: Int, ringPresent: Boolean) {

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
        // If we are being sent a ring result, then show that result
        val resultTextStartPoint = Point(5.0, mat.rows()-10.0)
                .times(resolution.scale)
                .coerceIn(mat)
        val resultText = "found $numSuccessful datapoints, indicating there ${if (ringPresent) "is" else "is not"} a ring"
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

}