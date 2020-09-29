package org.firstinspires.ftc.teamcode.library.vision.base

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.teamcode.library.functions.process.Stoppable
import org.firstinspires.ftc.teamcode.library.vision.skystone.SkystonePixelStatsPipeline
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline
import org.openftc.easyopencv.OpenCvWebcam

class OpenCvContainer<out Pipeline : OpenCvPipeline>(
        val camera: OpenCvCamera,
        val pipeline: Pipeline,
        private val resolution: ImageResolution,
        private val rotation: OpenCvCameraRotation = OpenCvCameraRotation.UPRIGHT) : Stoppable {

    init {
        camera.setPipeline(pipeline)
        camera.openCameraDevice()
        camera.openCameraDeviceAsync {
            camera.startStreaming(resolution.width, resolution.height, rotation)
        }
        camera.showFpsMeterOnViewport(false)
        if (pipeline is ResolutionPipeline) pipeline.resolution = resolution
    }

    fun adjustSettings(func: (OpenCvCamera)->Unit) {
        func(camera)
    }


    override fun stop() = camera.stopStreaming()
}