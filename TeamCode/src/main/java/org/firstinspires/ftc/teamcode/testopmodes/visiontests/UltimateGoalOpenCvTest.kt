package org.firstinspires.ftc.teamcode.testopmodes.visiontests

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.base.OpenCvContainer
import org.firstinspires.ftc.teamcode.library.vision.ultimategoal.RingContourPipeline

@TeleOp(group="Test")
class UltimateGoalOpenCvTest : OpMode() {

    lateinit var container : OpenCvContainer<RingContourPipeline>

    var cameraType = VisionFactory.CameraType.WEBCAM

    override fun init() {
        container = VisionFactory.createOpenCv(cameraType, hardwareMap,
                RingContourPipeline())
        container.pipeline.shouldKeepTracking = true
    }

    override fun start() {
        container.pipeline.tracking = true
    }

    override fun loop() {

        telemetry.addData("Last Result", container.pipeline.numberOfRings)
        telemetry.update()
    }
}