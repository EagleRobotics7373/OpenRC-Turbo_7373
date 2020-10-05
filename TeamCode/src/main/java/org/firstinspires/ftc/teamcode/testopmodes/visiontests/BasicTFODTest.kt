package org.firstinspires.ftc.teamcode.testopmodes.visiontests

import android.graphics.Point
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.functions.ToggleButtonWatcher
import org.firstinspires.ftc.teamcode.library.vision.base.TFODContainer
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory

@TeleOp
class BasicTFODTest: LinearOpMode() {
    override fun runOpMode() {
        val vuforia = VisionFactory.createVuforia(hardwareMap)
        val tfod = VisionFactory.attachTFOD(hardwareMap, vuforia)
        val tfodContainer = TFODContainer(vuforia, tfod)
        val gamepadButtonA = ToggleButtonWatcher { gamepad1.a }

        waitForStart()

        if (!opModeIsActive()) return

        while (opModeIsActive()) {
            if (gamepadButtonA.call()) {
                val recognitions = tfodContainer.getUpdatedRecognitions()
                recognitions.forEachIndexed { index, recognition ->
                    val no = "($index)"
                    telemetry.addLine("RECOGNITION $index")
                    telemetry.addData("label$no", recognition.label)
                    telemetry.addData("confidence$no", recognition.confidence)
                    telemetry.addData("top$no", recognition.top)
                    telemetry.addData("left$no", recognition.left)
                    telemetry.addData("bottom$no", recognition.bottom)
                    telemetry.addData("right$no", recognition.right)
                    telemetry.addData("height$no", recognition.height)
                    telemetry.addData("width$no", recognition.width)
                    telemetry.addLine("-----------")
                }
                telemetry.update()
            }
            else if (gamepad1.b) {
                telemetry.update()
            }

        }

        tfodContainer.stop()
    }
}