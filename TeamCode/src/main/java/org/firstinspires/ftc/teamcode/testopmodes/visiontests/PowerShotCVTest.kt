package org.firstinspires.ftc.teamcode.testopmodes.visiontests

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.base.OpenCvContainer
import org.firstinspires.ftc.teamcode.library.vision.ultimategoal.PowerShotPipeline
import org.firstinspires.ftc.teamcode.library.vision.ultimategoal.RingContourPipeline

@TeleOp(group="Test")
class PowerShotCVTest : OpMode() {

    lateinit var container : OpenCvContainer<PowerShotPipeline>

    var cameraType = VisionFactory.CameraType.WEBCAM_MINUS

    override fun init() {
        container = VisionFactory.createOpenCv(cameraType, hardwareMap,
                PowerShotPipeline())
        container.pipeline.shouldKeepTracking = true
    }

    override fun start() {
        container.pipeline.tracking = false
    }

    override fun loop() {

        if (gamepad1.a) container.pipeline.tracking = true
        if (gamepad1.b) container.pipeline.tracking = false
        container.pipeline.distances?.forEachIndexed {index,it->
            telemetry.addData("PS$index", it.toString())
        }
        telemetry.update()
    }
}