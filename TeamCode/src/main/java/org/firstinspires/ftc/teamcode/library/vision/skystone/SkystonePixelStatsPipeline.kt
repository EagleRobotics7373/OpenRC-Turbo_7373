package org.firstinspires.ftc.teamcode.library.vision.skystone

import org.firstinspires.ftc.teamcode.library.functions.Position
import org.firstinspires.ftc.teamcode.library.vision.base.ImageResolution
import org.firstinspires.ftc.teamcode.library.vision.base.ResolutionPipeline
import org.opencv.imgproc.Imgproc

import org.firstinspires.ftc.teamcode.library.vision.skystone.SkystoneVisionConstants.HS_FIRST_STONE_PT
import org.firstinspires.ftc.teamcode.library.vision.skystone.SkystoneVisionConstants.HS_STONE_GAP
import org.firstinspires.ftc.teamcode.library.vision.skystone.SkystoneVisionConstants.HS_STONE_WIDTH
import org.opencv.core.*
import org.opencv.imgproc.Imgproc.*
import kotlin.math.pow

class SkystonePixelStatsPipeline(var detector: StatsDetector) : ResolutionPipeline() {


    var skystonePos : Position? = null
        private set


    lateinit var rgbMat: Mat
    lateinit var hueMat: Mat

    override var resolution : ImageResolution = ImageResolution.R_720x480

    override fun processFrame(input: Mat): Mat {
        if (tracking) {
            val startingRuntime = System.currentTimeMillis()

            rgbMat = input

            extractHue()

            // track first stone
            val stats = arrayOf(
                trackStone(HS_FIRST_STONE_PT.x.toInt(), HS_FIRST_STONE_PT.y.toInt(), true),
                trackStone((HS_FIRST_STONE_PT.x + HS_STONE_WIDTH*2 + HS_STONE_GAP).toInt(), HS_FIRST_STONE_PT.y.toInt(), false)
            )

            stats.forEach {
                println("\tSTONE at x = ${it.x}, y = ${it.y} :" +
                        "\n\t\tAverage = ${it.average}" +
                        "\n\t\tStd.Dev = ${it.stdDev}" +
                        "\n\t\tSamples = ${it.samples}" +
                        "\n\t\tRuntime = ${it.runtime}")
            }

            val leftStoneProp = detector.obtainProperty(stats[0])
            val rightStoneProp = detector.obtainProperty(stats[1])

            skystonePos = when {
                (leftStoneProp > rightStoneProp && leftStoneProp-rightStoneProp>detector.differenceNeeded)  -> detector.leftGreaterThanRightResult
                (leftStoneProp < rightStoneProp && rightStoneProp-leftStoneProp>detector.differenceNeeded)  -> detector.rightGreaterThanLeftResult
                else                                                                                        -> Position.NULL
            }


            addLabels()

            putText(hueMat, "Skystone " +
                    when(skystonePos) {
                        Position.LEFT-> "on LEFT of frame"
                        Position.RIGHT->"on RIGHT of frame"
                        else->"not visible"
                    },
                    Point(10.0 * resolution.scale, hueMat.rows()*0.98), FONT_HERSHEY_DUPLEX, 0.6*resolution.scale, Scalar(0.0), 1)

//            Imgcodecs.imwrite("/sdcard/FIRST/cvtrials/cvtrial_${resolution}_${System.currentTimeMillis()}_", hueMat)
            if (!shouldKeepTracking) tracking = false
            return hueMat

        } else return input
    }

    private fun extractHue() {
        val hsv = rgbMat.clone()
        Imgproc.cvtColor(rgbMat, hsv, Imgproc.COLOR_BGR2HSV)

        hueMat = Mat(hsv.rows(), hsv.cols(), CvType.CV_8UC1)
        Core.extractChannel(hsv, hueMat, detector.channelToExtract)
    }

    private fun trackStone(x: Int, y: Int, isLeft: Boolean) : StoneStats {
        val startingRuntime = System.currentTimeMillis()

        val stoneHeight = (HS_STONE_WIDTH * 0.48).toInt() // found from known stone height-to-width ratio

        val maxRows = hueMat.rows()
        val maxCols = hueMat.cols()

        val xMod = (x * resolution.scale).toInt()
        val yMod = (y * resolution.scale).toInt()

        // find bounding box
        val rowMin = (yMod - (stoneHeight*resolution.scale).toInt())   .coerceIn(0, maxRows-1)
        val rowMax = (yMod + (stoneHeight*resolution.scale).toInt())   .coerceIn(0, maxRows-1)
        val colMin = (xMod - (HS_STONE_WIDTH*resolution.scale).toInt()).coerceIn(0, maxCols-1)
        val colMax = (xMod + (HS_STONE_WIDTH*resolution.scale).toInt()).coerceIn(0, maxCols-1)

        // sample for average, then compute
        var ptsSum = 0.0
        var numSamples = 0
        for (rowInc in rowMin..rowMax)
            for (colInc in colMin..colMax) {
                ptsSum += hueMat.get(rowInc, colInc)[0]
                numSamples++
            }
        val average = ptsSum / numSamples

        // sample for std.dev, then compute
        var summationOfDeviations = 0.0
        var numberOfDeviationSums = 0
        for (rowInc in rowMin..rowMax)
            for (colInc in colMin..colMax) {
                summationOfDeviations += (hueMat.get(rowInc, colInc)[0] - average).pow(2)
                numberOfDeviationSums ++
            }
        val stdDev = (summationOfDeviations / (numberOfDeviationSums-1)).pow(0.5)

        // create visual bounding boxes
        rectangle(
                hueMat,
                Point(colMin.toDouble(), rowMin.toDouble()),
                Point(colMax.toDouble(), rowMax.toDouble()),
                Scalar(255.0),
                5
        )
        circle(
                hueMat,
                Point(xMod.toDouble(), yMod.toDouble()),
                7,
                Scalar(255.0),
                10.times(resolution.scale).toInt()
        )
        circle( // bottom left
                hueMat,
                Point(colMin.toDouble(), rowMin.toDouble()),
                7,
                Scalar(255.0),
                10.times(resolution.scale).toInt()
        )
        circle( // top left
                hueMat,
                Point(colMin.toDouble(), rowMax.toDouble()),
                7,
                Scalar(255.0),
                10.times(resolution.scale).toInt()
        )
        circle( // bottom right
                hueMat,
                Point(colMax.toDouble(), rowMin.toDouble()),
                7,
                Scalar(255.0),
                10.times(resolution.scale).toInt()
        )
        circle( // top right
                hueMat,
                Point(colMax.toDouble(), rowMax.toDouble()),
                7,
                Scalar(255.0),
                10.times(resolution.scale).toInt()
        )

        val detailsAt = if (isLeft) 0.51 else 0.75
        
        putText(hueMat, if (isLeft) "LEFT Sample" else "RIGHT Sample", Point(hueMat.cols()*detailsAt, hueMat.rows()*0.84), FONT_HERSHEY_DUPLEX, 0.6*resolution.scale, Scalar(0.0), 2)
        putText(hueMat, "Samples: $numSamples", Point(hueMat.cols()*detailsAt, hueMat.rows()*0.875), FONT_HERSHEY_DUPLEX, 0.6*resolution.scale, Scalar(0.0), 1)
        putText(hueMat, "Average: ${average.toInt()}", Point(hueMat.cols()*detailsAt, hueMat.rows()*0.91), FONT_HERSHEY_DUPLEX, 0.6*resolution.scale, Scalar(0.0), 1)
        putText(hueMat, "Std.Dev.: ${stdDev.toInt()}", Point(hueMat.cols()*detailsAt, hueMat.rows()*0.945), FONT_HERSHEY_DUPLEX, 0.6*resolution.scale, Scalar(0.0), 1)

        putText(hueMat, "Likely: " +
                when (average) {
                    in detector.skystoneValuesRange -> "Skystone"
                    in detector.stoneValuesRange -> "Stone"
                    else           -> "n/a"
                },
                Point(hueMat.cols()*detailsAt, hueMat.rows()*0.98), FONT_HERSHEY_DUPLEX, 0.6*resolution.scale, Scalar(0.0), 1);
        
        // find final runtime and return stats
        val elapsedRuntime = System.currentTimeMillis() - startingRuntime
        return StoneStats(x, y, average, stdDev, numSamples, elapsedRuntime)
    }

    private fun addLabels() {
        putText(hueMat, "Eagle Robotics Team 7373", Point(10 * resolution.scale, hueMat.rows()*0.84), FONT_HERSHEY_DUPLEX, 0.75*resolution.scale, Scalar(0.0), 2)
        putText(hueMat, "OpenCV Skystone Recognition", Point(10.0 * resolution.scale, hueMat.rows()*0.88), FONT_HERSHEY_DUPLEX, 0.6*resolution.scale, Scalar(0.0), 1)
        putText(hueMat, "Average/Std.Dev. Detector (Alpha)", Point(10.0 * resolution.scale, hueMat.rows()*0.92), FONT_HERSHEY_DUPLEX, 0.6*resolution.scale, Scalar(0.0), 1)

        line(hueMat, Point(hueMat.cols()*0.01, hueMat.rows()*0.94), Point(hueMat.cols()*0.475, hueMat.rows()*0.94), Scalar(0.0), 1)
    }

    class StoneStats(
            val x      : Int,
            val y      : Int,
            val average: Double,
            val stdDev : Double,
            val samples: Int,
            val runtime: Long
    )

    enum class StatsDetector(
            val channelToExtract: Int,
            val obtainProperty: (StoneStats)->Double,
            val differenceNeeded: Int,
            val skystoneValuesRange: ClosedFloatingPointRange<Double>,
            val stoneValuesRange: ClosedFloatingPointRange<Double>,
            val leftGreaterThanRightResult: Position,

            val rightGreaterThanLeftResult: Position) {

        DETECTOR_HUE_AVG(0, {it.average}, 20, 90.0..190.0, 20.0..90.0, Position.LEFT, Position.RIGHT),
        DETECTOR_HUE_STDDEV(0, {it.stdDev}, 5, 10.0..100.0, 0.0..10.0, Position.LEFT, Position.RIGHT),
        DETECTOR_VALUE_AVG(2, {it.average}, 50, 0.0..100.0, 100.0..255.0, Position.RIGHT, Position.LEFT),
        DETECTOR_VALUE_STDDEV(2, {it.stdDev}, 15, 27.0..100.0, 0.0..27.0, Position.LEFT, Position.RIGHT)
    }

}