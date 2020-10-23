package org.firstinspires.ftc.teamcode.testopmodes.visiontests

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.base.OpenCvContainer
import org.firstinspires.ftc.teamcode.library.vision.ultimategoal.IntakeRingViewingPipeline

@TeleOp(group="Test")
class UltimateGoalOpenCvTestBox : OpMode() {

    lateinit var container : OpenCvContainer<IntakeRingViewingPipeline>

    var cameraType = VisionFactory.CameraType.WEBCAM

    override fun init() {
        container = VisionFactory.createOpenCv(cameraType, hardwareMap,
                IntakeRingViewingPipeline())
        container.pipeline.shouldKeepTracking = true
    }

    override fun start() {
        container.pipeline.tracking = true
    }

    override fun loop() {

        telemetry.addData("Last Result", container.pipeline.numSuccessfulOutsideIntake)
        telemetry.update()
    }
}