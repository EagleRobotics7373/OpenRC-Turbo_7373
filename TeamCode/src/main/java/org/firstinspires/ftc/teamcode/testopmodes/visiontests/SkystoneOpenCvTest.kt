package org.firstinspires.ftc.teamcode.testopmodes.visiontests

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.functions.Position
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.skystone.SkystonePixelStatsPipeline
import org.firstinspires.ftc.teamcode.library.vision.base.OpenCvContainer

@TeleOp(group="Test")
class SkystoneOpenCvTest : OpMode() {

    lateinit var container : OpenCvContainer<SkystonePixelStatsPipeline>

    var cameraType = VisionFactory.CameraType.PHONE_REAR
    var currentlyTracking = false
    var visionResult: Position? = null

    val detectorOptions = arrayOf(
            SkystonePixelStatsPipeline.StatsDetector.DETECTOR_HUE_AVG,
            SkystonePixelStatsPipeline.StatsDetector.DETECTOR_HUE_STDDEV,
            SkystonePixelStatsPipeline.StatsDetector.DETECTOR_VALUE_AVG,
            SkystonePixelStatsPipeline.StatsDetector.DETECTOR_VALUE_STDDEV
            )
    var detectorNum = 0

    override fun init() {
    }

    override fun init_loop() {
        if (gamepad1.x) cameraType = VisionFactory.CameraType.WEBCAM
        else if (gamepad1.a) cameraType = VisionFactory.CameraType.PHONE_FRONT
        else if (gamepad1.b) cameraType = VisionFactory.CameraType.PHONE_REAR

        telemetry.addData("Camera", cameraType)
        telemetry.update()
    }

    override fun start() {
        container = VisionFactory.createOpenCv(cameraType, hardwareMap, SkystonePixelStatsPipeline(SkystonePixelStatsPipeline.StatsDetector.DETECTOR_HUE_AVG))
        container.pipeline.shouldKeepTracking = true
        container.pipeline.tracking = true
    }

    override fun loop() {
        if (!currentlyTracking) {
            telemetry.addData("Status", "Not Tracking")
            if (gamepad1.y) {
                container.pipeline.tracking = true
                currentlyTracking = true
//                while (gamepad1.y);
            }
        }
        else {
            telemetry.addData("Status", "Tracking")
            if (!container.pipeline.tracking)
                currentlyTracking = false
                visionResult = container.pipeline.skystonePos
        }
        if (gamepad1.b) visionResult = null

        if (gamepad1.dpad_down) {
            detectorNum--
            while(gamepad1.dpad_down);
        } else if (gamepad1.dpad_up) {
            detectorNum++
            while(gamepad1.dpad_up);
        }

        container.pipeline.detector = detectorOptions[detectorNum]

        telemetry.addData("Last Result", visionResult)
        telemetry.addData("Detector", detectorOptions[detectorNum])
        telemetry.addData("Detector Num", detectorNum)
        telemetry.update()
    }
}