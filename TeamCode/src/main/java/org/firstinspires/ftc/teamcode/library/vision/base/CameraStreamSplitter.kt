package org.firstinspires.ftc.teamcode.library.vision.base

import com.acmerobotics.dashboard.FtcDashboard
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource

object CameraStreamSplitter {
    fun startCameraStream(source: CameraStreamSource, maxFps: Double = 15.0) {
        CameraStreamServer.getInstance().setSource(source)
        FtcDashboard.getInstance().startCameraStream(source, maxFps)
    }

    fun stopCameraStream() {
        CameraStreamServer.getInstance().setSource(null)
        FtcDashboard.getInstance().stopCameraStream()
    }
}