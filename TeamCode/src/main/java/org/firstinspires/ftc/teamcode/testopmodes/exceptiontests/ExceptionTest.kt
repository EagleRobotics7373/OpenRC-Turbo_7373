package org.firstinspires.ftc.teamcode.testopmodes.exceptiontests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.functions.ToggleButtonWatcher
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin.*
import org.firstinspires.ftc.teamcode.library.vision.base.OpenCvContainer
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.ultimategoal.IntakeRingViewingPipeline
import org.opencv.core.CvException
import java.lang.ArithmeticException
import java.lang.Exception
import kotlin.concurrent.thread

@TeleOp(name="Robot Crash Utility", group="Danger")
class ExceptionTest : LinearOpMode() {

    enum class CrashLocation {
        DEFAULT, NEW_THREAD, SAFE_THREAD, VISION
    }

    val menu = DelegatedTelemetryMenu(telemetry)
    var exception: Exception by MenuItemEnumDelegate("Exception",
            IllegalStateException(),
            ArithmeticException(),
            CvException("something bad with opencv"),
            RuntimeException(),
            NullPointerException(),
            Exception()
    ) with menu
    var location: CrashLocation by MenuItemEnumDelegate("Crash Location", *CrashLocation.values()) with menu


    lateinit var dpadUpWatch : ToggleButtonWatcher
    lateinit var dpadDownWatch : ToggleButtonWatcher
    lateinit var dpadLeftWatch : ToggleButtonWatcher
    lateinit var dpadRightWatch : ToggleButtonWatcher

    lateinit var container: OpenCvContainer<IntakeRingViewingPipeline>

    override fun runOpMode() {
        container = VisionFactory.createOpenCv(VisionFactory.CameraType.WEBCAM, hardwareMap, IntakeRingViewingPipeline())

        dpadUpWatch = ToggleButtonWatcher {gamepad1.dpad_up}
        dpadDownWatch = ToggleButtonWatcher {gamepad1.dpad_down}
        dpadLeftWatch = ToggleButtonWatcher {gamepad1.dpad_left}
        dpadRightWatch = ToggleButtonWatcher {gamepad1.dpad_right}
        while (!isStarted && !isStopRequested) {
            when {
                dpadUpWatch.invoke()    -> menu.previousItem()
                dpadDownWatch.invoke()  -> menu.nextItem()
                dpadLeftWatch.invoke()  -> menu.iterateBackward()
                dpadRightWatch.invoke() -> menu.iterateForward()
                else -> menu.refresh()
            }
        }


        when (location) {
            CrashLocation.DEFAULT -> throw exception
            CrashLocation.SAFE_THREAD -> thread {
                try {
                    throw exception
                }
                finally {
                    exception.printStackTrace()
                }
            }
            CrashLocation.NEW_THREAD -> thread { throw exception }
            CrashLocation.VISION -> container.pipeline.exception = exception
        }
        while (opModeIsActive()) {
            if (gamepad1.a) break
        }
    }

}