package org.firstinspires.ftc.teamcode.library.vision.ultimategoal

import org.firstinspires.ftc.teamcode.library.functions.truncate
import org.firstinspires.ftc.teamcode.library.vision.base.ImageResolution
import org.firstinspires.ftc.teamcode.library.vision.base.ResolutionPipeline
import org.firstinspires.ftc.teamcode.library.vision.base.coerceIn
import org.firstinspires.ftc.teamcode.library.vision.base.times
import org.firstinspires.ftc.teamcode.library.vision.ultimategoal.UltimateGoalVisionConstants.*

import org.opencv.core.*
import org.opencv.core.Core.*
import org.opencv.imgproc.Imgproc
import org.opencv.imgproc.Imgproc.*
import kotlin.math.pow


class RingContourPipeline() : ResolutionPipeline() {

    // Public variable allowing OpModes to access number of rings
    var numberOfRings : Int? = null
        private set

    // The mat that we will analyze, alone with the channel number we need to extract from HSV
    private var hlsMat: Mat = Mat()
    private var thresholdResult: Mat = Mat()

    override var resolution : ImageResolution = ImageResolution.R_720x480

    override fun processFrame(input: Mat): Mat {
        if (tracking) {

            // Blur the image for greater contour recognition accuracy
            medianBlur(input, input, 15)

            // Convert our input, in BGR format, to HLS (hue, luminosity, saturation)
            cvtColor(input, hlsMat, COLOR_BGR2HLS)

            // Threshold the HLS mat to only include objects with yellow hue and high saturation
            inRange(
                    hlsMat,                             // original mat
                    Scalar(                             // lower bound for threshold
                            CONTOUR_HUE_LOWER_BOUND,
                            CONTOUR_LUM_LOWER_BOUND,
                            CONTOUR_SAT_LOWER_BOUND),
                    Scalar(                             // upper bound for threshold
                            CONTOUR_HUE_UPPER_BOUND,
                            CONTOUR_LUM_UPPER_BOUND,
                            CONTOUR_SAT_UPPER_BOUND),
                    thresholdResult                     // resultant mat
            )

            val elementType = Imgproc.CV_SHAPE_RECT;
            val kernelSize = 5.0;
            val element = getStructuringElement(
                    elementType, Size(2 * kernelSize + 1, 2 * kernelSize + 1),
                    Point(kernelSize, kernelSize)
            )
            erode(
                    thresholdResult,                    // original mat
                    thresholdResult,                    // resultant mat - just overwrite the original
                    element                             // iterations - more of these means more erosion
            )


            val contours = emptyList<MatOfPoint>().toMutableList()
            val hierarchy = Mat()
            findContours(
                    thresholdResult,
                    contours,
                    hierarchy,
                    RETR_EXTERNAL,
                    CHAIN_APPROX_SIMPLE
            )

            val res = contours.mapIndexed { index, matOfPoint ->

                drawContours(
                        input,
                        contours,
                        index,
                        Scalar.all(150.0),
                        5
                )

                var minX = Int.MAX_VALUE
                var minY = Int.MAX_VALUE
                var maxX = Int.MIN_VALUE
                var maxY = Int.MIN_VALUE
                for (point in matOfPoint.toArray()) {
                    if (point.x < minX) minX = point.x.toInt()
                    if (point.y < minY) minY = point.y.toInt()
                    if (point.x > maxX) maxX = point.x.toInt()
                    if (point.y > maxY) maxY = point.y.toInt()
                }

                println("CONTOUR $index min=($minX, $minY) max=($maxX, $maxY)")

                return@mapIndexed ContourResult(
                        Point(minX.toDouble(), minY.toDouble()),
                        Point(maxX.toDouble(), maxY.toDouble()))
            }.filter { it.ratio in 1.0..3.0 }.maxBy { it.area }?.ratio

            numberOfRings = when (res) {
                null -> 0
                in 1.00..1.75 -> 1
                in 1.75..3.00 -> 4
                else -> 0
            }
            addLabels(input)
            return input
        }
        else {
            addLabels(input)
            return input
        }
    }

    class ContourResult(
            val min: Point,
            val max: Point
    )  {
        val width = max.x - min.x
        val height = max.y - min.y
        val ratio = width.toDouble() / height
        val area = width * height
    }

    private fun inverseColorAtPoint(mat: Mat, point: Point): DoubleArray {
        val newPoint = point.coerceIn(mat)
        return mat.get(newPoint.y.toInt(), newPoint.x.toInt()) // Get color at this point
                .map { 255 - it }                              // For each channel, invert the color
//                .dropLast(1)
                .toDoubleArray()                               // Re-convert to DoubleArray
    }


    private fun addLabels(mat: Mat) {

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
        putText(mat, "ULTIMATE GOAL CV Ring Contour Detection Pipeline", detectorNameStartPoint,
                FONT_HERSHEY_SIMPLEX, 0.55 * resolution.scale, 
                Scalar(inverseColorAtPoint(mat, detectorNameStartPoint)), 2)

        // For each ring, show the RingStats found during image processing

        // If we are being sent a ring result, then show that result
        val resultTextStartPoint = Point(5.0, mat.rows()-10.0)
                .times(resolution.scale)
                .coerceIn(mat)
        val resultText = when {
            numberOfRings == null   -> "No detection stored."
            else                    -> "Detecting $numberOfRings rings."
        }
        putText(mat, resultText,
                resultTextStartPoint,
                FONT_HERSHEY_SIMPLEX, 0.6 * resolution.scale,
                Scalar(inverseColorAtPoint(mat, resultTextStartPoint)), 2)
    }




}