package org.firstinspires.ftc.teamcode.library.vision.ultimategoal

import org.firstinspires.ftc.teamcode.library.functions.AllianceColor
import org.firstinspires.ftc.teamcode.library.vision.base.ImageResolution
import org.firstinspires.ftc.teamcode.library.vision.base.ResolutionPipeline
import org.firstinspires.ftc.teamcode.library.vision.base.coerceIn
import org.firstinspires.ftc.teamcode.library.vision.base.times
import org.firstinspires.ftc.teamcode.library.vision.ultimategoal.UltimateGoalPowerShotConstants.*
import org.opencv.core.*
import org.opencv.core.Core.*
import org.opencv.imgproc.Imgproc
import org.opencv.imgproc.Imgproc.*


class PowerShotPipeline() : ResolutionPipeline() {

    // Public variable allowing OpModes to access data
    var distances: List<Distance>? = null
        private set

    // The mat that we will analyze, alone with the channel number we need to extract from HSV
    private var subMat: Mat = Mat()
    private var hsvMat: Mat = Mat()
    private var threshLow: Mat = Mat()
    private var threshHigh: Mat = Mat()
    private var threshResult: Mat = Mat()

    override var resolution : ImageResolution = ImageResolution.R_720x480

    override fun processFrame(input: Mat): Mat {
        if (input.empty()) return input

        if (tracking) {

            subMat = input.submat(CUTOFF_TOP, CUTOFF_BOTTOM, CUTOFF_LEFT, CUTOFF_RIGHT)

            // Convert our input, in RGB format, to HLS (hue, luminosity, saturation)
            cvtColor(subMat, hsvMat, COLOR_RGB2HSV)

            // Blur the image for greater contour recognition accuracy
            when (BLUR_TYPE) {
                PixelAction.BLUR_MEDIAN -> medianBlur(hsvMat, hsvMat, BLUR_MEDIAN_SIZE)
                PixelAction.BLUR -> blur(hsvMat, hsvMat, Size(BLUR_WIDTH, BLUR_HEIGHT))
            }

            // Threshold image based on alliance color
            when (ALLIANCE_COLOR) {
                AllianceColor.RED -> {
                    // Threshold image to get lower red->orange hues
                    inRange(
                            hsvMat,
                            Scalar(0.0, RED_SAT_LOW.toDouble(), VALUE_LOW.toDouble()),
                            Scalar(RED_HUE_LOW.toDouble(), RED_SAT_HIGH.toDouble(), VALUE_HIGH.toDouble()),
                            threshLow
                    )
                    // Threshold image to get upper pink->red hues
                    inRange(
                            hsvMat,
                            Scalar(RED_HUE_HIGH.toDouble(), RED_SAT_LOW.toDouble(), VALUE_LOW.toDouble()),
                            Scalar(180.0, RED_SAT_HIGH.toDouble(), VALUE_HIGH.toDouble()),
                            threshHigh
                    )
                    // Combine threshold results
                    bitwise_or(threshLow, threshHigh, threshResult)
                }
                else -> {
                    // Threshold image to get blue hues
                    inRange(
                            hsvMat,
                            Scalar(BLUE_HUE_LOW.toDouble(), BLUE_SAT_LOW.toDouble(), VALUE_LOW.toDouble()),
                            Scalar(BLUE_HUE_HIGH.toDouble(), BLUE_SAT_HIGH.toDouble(), VALUE_HIGH.toDouble()),
                            threshResult
                    )
                }
            }

            // Create erosion kernel
            val elementType = Imgproc.CV_SHAPE_RECT;
            val kernelHeight = KERNEL_HEIGHT.toDouble()
            val kernelWidth = KERNEL_WIDTH.toDouble()
            val element = getStructuringElement(
                    elementType, Size(2 * kernelWidth + 1, 2 * kernelHeight + 1),
                    Point(kernelWidth, kernelHeight)
            )

            // Erode or dilate the image
            when (PIXEL_EXPANSION_ACTION) {
                PixelAction.ERODE -> erode(threshResult, threshResult, element)
                PixelAction.DILATE -> dilate(threshResult, threshResult, element)
            }


            // TODO: Determine output image here?

            // Get image contours
            val contours = emptyList<MatOfPoint>().toMutableList()
            findContours(
                    threshResult,
                    contours,
                    Mat(),
                    RETR_EXTERNAL,
                    CHAIN_APPROX_SIMPLE
            )

            // Get contours and do basic filtering and drawing operations
            val contoursInfo = contours
                    .mapIndexed { index, contour -> ContourInfo(contour, index) }
                    .filter { it.area > AREA_CUTOFF && it.ratio in ASPECT_LOW_CUTOFF..ASPECT_HIGH_CUTOFF }
                    .onEach {

                        putText(subMat, String.format("%.2f@%d@c%d", it.ratio, it.area, it.index),
                                it.minPoint, FONT_HERSHEY_SIMPLEX, 0.5,
                                inverseColorAtPoint(subMat, it.minPoint).toScalar(), 1)
                        rectangle(subMat, it.minPoint, it.maxPoint,
                                inverseColorAtPoint(subMat, it.minPoint).toScalar())
                        line(subMat, Point(it.midPoint.x, 0.0), Point(it.midPoint.x, subMat.rows() - 1.0),
                                inverseColorAtPoint(subMat, it.midPoint).toScalar(), 2)

                    }
                    .sortedBy { it.minPoint.x }

            if (contoursInfo.size >= 3) {

                val threeContours = (
                        if (ALLIANCE_COLOR == AllianceColor.RED) contoursInfo.take(3)
                        else contoursInfo.takeLast(3))

                val pixelsBetweenContours = threeContours[1].minPoint.x - threeContours[0].minPoint.x
                val pixelsPerInch = pixelsBetweenContours / 7.5

                distances = threeContours.map {
                    Distance((it.midPoint.x - GET_MODIFIED_TARGET_X()) / pixelsPerInch)
                }

            } else {
                distances = null
            }

            line(subMat,
                    Point(GET_MODIFIED_TARGET_X().toDouble(), 0.0),
                    Point(GET_MODIFIED_TARGET_X().toDouble(), subMat.rows().toDouble() - 1),
                    Scalar(0.0, 220.0, 0.0), 4)

            return if (RETURN_SINGLE_CHANNEL) threshResult else subMat

        }

        return input
    }

    class ContourInfo
    constructor(contour: MatOfPoint, val index: Int)
    {

        val left: Int
        val right: Int
        val top: Int
        val bottom: Int

        val width: Int
        val height: Int

        init {
            var minX = Int.MAX_VALUE
            var minY = Int.MAX_VALUE
            var maxX = Int.MIN_VALUE
            var maxY = Int.MIN_VALUE
            for (point in contour.toArray()) {
                if (point.x < minX) minX = point.x.toInt()
                if (point.y < minY) minY = point.y.toInt()
                if (point.x > maxX) maxX = point.x.toInt()
                if (point.y > maxY) maxY = point.y.toInt()
            }

            left = minX
            right = maxX
            top = minY
            bottom = maxY

            width = right - left
            height = bottom - top
        }

        val minPoint = Point(left.toDouble(), top.toDouble())
        val maxPoint = Point(right.toDouble(), bottom.toDouble())

        val midPoint = Point((left + right) / 2.0, (top+bottom)/2.0)

        val ratio = width.toDouble() / height
        val area = width * height

    }

    class Distance(val distance: Double) {
        val suggestion = when {
            distance < -SAFE_SHOT -> "<-- MOVE LEFT"
            distance >  SAFE_SHOT -> "--> MOVE RIGHT"
            else                  -> "SHOOT NOW!"
        }

        override fun toString(): String = String.format("%.1f\t$suggestion", distance)
    }

    private fun inverseColorAtPoint(mat: Mat, point: Point): DoubleArray {
        val newPoint = point.coerceIn(mat)
        return mat.get(newPoint.y.toInt(), newPoint.x.toInt()) // Get color at this point
                .map { 255 - it }                              // For each channel, invert the color
//                .dropLast(1)
                .toDoubleArray()                               // Re-convert to DoubleArray
    }


    private fun addLabels(mat: Mat, ratio: Double? = null) {

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
        putText(mat, "ULTIMATE GOAL CV Power Shot Detection Pipeline", detectorNameStartPoint,
                FONT_HERSHEY_SIMPLEX, 0.55 * resolution.scale,
                Scalar(inverseColorAtPoint(mat, detectorNameStartPoint)), 2)

        // For each ring, show the RingStats found during image processing

        // If we are being sent a ring result, then show that result
//        val resultTextStartPoint = Point(5.0, 30.0)
//                .coerceIn(mat)
//        val resultText = when {
//            numberOfRings == null   -> "No detection stored."
//            ratio == null           -> "Detecting $numberOfRings rings."
//            else                    -> "Detecting $numberOfRings rings with ratio ${String.format("%.2f", ratio)}"
//        }
//        putText(mat, resultText,
//                resultTextStartPoint,
//                FONT_HERSHEY_SIMPLEX, 0.6 * resolution.scale,
//                Scalar(inverseColorAtPoint(mat, resultTextStartPoint)), 2)
    }

    private fun DoubleArray.toScalar() = Scalar(this)




}