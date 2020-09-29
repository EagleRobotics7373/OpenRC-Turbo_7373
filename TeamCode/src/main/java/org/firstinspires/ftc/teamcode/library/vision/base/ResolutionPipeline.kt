package org.firstinspires.ftc.teamcode.library.vision.base

import org.openftc.easyopencv.OpenCvPipeline

abstract class ResolutionPipeline: OpenCvPipeline() {
    abstract var resolution: ImageResolution
    var shouldKeepTracking = false
    var tracking : Boolean = false
}