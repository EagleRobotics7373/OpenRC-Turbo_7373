package org.firstinspires.ftc.teamcode.library.vision.base

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector
import org.firstinspires.ftc.teamcode.library.functions.process.Stoppable

class TFODContainer(
        val vuforia: VuforiaLocalizer,
        val tfod: TFObjectDetector) : Stoppable
{

    init {
        CameraStreamSplitter.startCameraStream(tfod)
    }

    fun getUpdatedRecognitions(): List<Recognition>? = tfod.updatedRecognitions

    fun activate() {
        tfod.activate()
    }

    override fun stop() {
        CameraStreamSplitter.stopCameraStream()
        tfod.deactivate()
        tfod.shutdown()
    }
}