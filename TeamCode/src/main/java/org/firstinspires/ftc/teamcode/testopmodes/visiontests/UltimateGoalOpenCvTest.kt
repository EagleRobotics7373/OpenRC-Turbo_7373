package org.firstinspires.ftc.teamcode.testopmodes.visiontests

import android.view.FocusFinder
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.base.OpenCvContainer
import org.firstinspires.ftc.teamcode.library.vision.ultimategoal.RingPixelAnalysisPipeline
import org.firstinspires.ftc.teamcode.library.vision.ultimategoal.UltimateGoalVisionConstants
import org.openftc.easyopencv.OpenCvWebcam
import java.util.concurrent.TimeUnit

@TeleOp(group="Test")
class UltimateGoalOpenCvTest : OpMode() {

    lateinit var container : OpenCvContainer<RingPixelAnalysisPipeline>

    var cameraType = VisionFactory.CameraType.WEBCAM

    override fun init() {
        container = VisionFactory.createOpenCv(cameraType, hardwareMap,
                RingPixelAnalysisPipeline())
        container.pipeline.shouldKeepTracking = true
    }

    override fun start() {
        container.pipeline.tracking = true
    }

    override fun loop() {
        container.adjustSettings {
            val webcam = it as? OpenCvWebcam

            val exposureControl = webcam?.exposureControl
            telemetry.addData("Exposure control null", exposureControl == null)
            telemetry.addData("Exposure control allowed", exposureControl?.isExposureSupported)
            telemetry.addData("Exposure control mode allowed", exposureControl?.isModeSupported(UltimateGoalVisionConstants.exposureMode))
            telemetry.addData("Exposure control min exposure", exposureControl?.getMinExposure(TimeUnit.MILLISECONDS))
            telemetry.addData("Exposure control max exposure", exposureControl?.getMaxExposure(TimeUnit.MILLISECONDS))


            val focusControl = webcam?.focusControl
            telemetry.addData("Focus control null", focusControl == null)
            telemetry.addData("Focus control available", focusControl?.isFocusLengthSupported)
            telemetry.addData("Min focus length", focusControl?.minFocusLength)
            telemetry.addData("Max focus length", focusControl?.maxFocusLength)
            telemetry.addData("Focus length mode", focusControl?.mode)


            if (gamepad1.x) {
                exposureControl?.mode = UltimateGoalVisionConstants.exposureMode
                exposureControl?.setExposure(UltimateGoalVisionConstants.exposure.toLong(), TimeUnit.MILLISECONDS)
                focusControl?.mode =UltimateGoalVisionConstants.mode
                focusControl?.focusLength = UltimateGoalVisionConstants.focusLength
            }


        }

        telemetry.addData("Last Result", container.pipeline.numberOfRings)
        telemetry.update()
    }
}