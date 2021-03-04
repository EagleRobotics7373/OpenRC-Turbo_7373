package org.firstinspires.ftc.teamcode.opmodes.gen2

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.*
import org.firstinspires.ftc.teamcode.library.functions.StartingLine.*
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtZoomBot
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtZoomBotConstants
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.WobbleGrabber
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.base.OpenCvContainer
import org.firstinspires.ftc.teamcode.library.vision.ultimategoal.PowerShotPipeline
import org.firstinspires.ftc.teamcode.opmodes.gen1b.OpModeConfig

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "DEMO: Power Shot TeleOp Test", group = "Main")
class PowerSeekTest : LinearOpMode() {

    /*
        VARIABLES: Hardware and Control
     */
    private lateinit var robot           : ExtZoomBot
    private lateinit var imuController   : IMUController

    private          val telem           : MultipleTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    private lateinit var elapsedTime     : ElapsedTime

    private lateinit var cvContainer     : OpenCvContainer<PowerShotPipeline>

    private lateinit var player          : ExtDirMusicPlayer

    /*
        VARIABLES: Menu Options
     */
    private val config = OpModeConfig(telemetry)
    private var allianceColor: AllianceColor by config.custom("Alliance Color", RED, BLUE)
    private var startingLine: StartingLine by config.custom("Starting Line", FAR, CENTER)

    override fun runOpMode() {
        /*
            Main autonomous variable initialization
         */
        robot = ExtZoomBot(hardwareMap)
        imuController = robot.imuControllerC
        cvContainer = VisionFactory.createOpenCv(
                VisionFactory.CameraType.WEBCAM_PLUS,
                hardwareMap,
                PowerShotPipeline())
        cvContainer.pipeline.shouldKeepTracking = true
        cvContainer.pipeline.tracking = true
        robot.zoomWheel.mode = DcMotor.RunMode.RUN_USING_ENCODER
        robot.zoomWheel.velocity = 0.0
        var targetPS = 0;


        /*
            Operate reflective telemetry menu, releasing when OpMode starts or stops
         */
        operateMenu()


        /*
            OpMode has proceeded past init. If stop is requested, return runOpMode() early.
         */
        if (isStopRequested) return


        /*
            Prepare music and setup additional components.
         */
        robot.holonomicRR.redefine()



        var lastError = 0.0
        while(opModeIsActive()) {
            val current: Double = (cvContainer.pipeline.distances?.get(targetPS)?.distance) ?: 0.0
            val target = 0.0
            val error = target - current
            val pOutput = error * -ExtZoomBotConstants.PS_SEEK.p
            val dOutput = (error - lastError) * ExtZoomBotConstants.PS_SEEK.d
            val output = pOutput + dOutput
            val actualOutput =
                    if (output < 0) output.coerceIn(-ExtZoomBotConstants.PS_SEEK_MAX, -ExtZoomBotConstants.PS_SEEK_MIN)
                    else if (output > 0) output.coerceIn(ExtZoomBotConstants.PS_SEEK_MIN, ExtZoomBotConstants.PS_SEEK_MAX)
                    else 0.0


            telemetry.addData("error", error)
            telemetry.addData("last error", lastError)
            telemetry.addData("p output", pOutput)
            telemetry.addData("d output", dOutput)
            telemetry.addData("output", output)
            telemetry.addData("actual output", actualOutput)
            telemetry.addData("target PS", targetPS)
            telemetry.addData("LEFT", cvContainer.pipeline.distances?.get(0))
            telemetry.addData("CENTER", cvContainer.pipeline.distances?.get(1))
            telemetry.addData("RIGHT", cvContainer.pipeline.distances?.get(2))
            telemetry.update()

            if (lastError != error) lastError = error

            if (gamepad1.x) {
                robot.frontLeftMotor.power = if (ExtZoomBotConstants.PS_FRONT_LEFT) actualOutput else 0.0
                robot.frontRightMotor.power = if (ExtZoomBotConstants.PS_FRONT_RIGHT) actualOutput else 0.0
                robot.backLeftMotor.power = if (ExtZoomBotConstants.PS_BACK_LEFT) actualOutput else 0.0
                robot.backRightMotor.power = if (ExtZoomBotConstants.PS_BACK_RIGHT) actualOutput else 0.0
            } else {
                robot.holonomic.stop()
            }

            when {
                gamepad1.dpad_left  -> targetPS = 0
                gamepad1.dpad_up    -> targetPS = 1
                gamepad1.dpad_right -> targetPS = 2
                gamepad1.a          -> robot.zoomWheel.velocity = ExtZoomBotConstants.AUTO_TEST_2
                gamepad1.b          -> robot.zoomWheel.velocity = 0.0
                gamepad1.y          -> {
                    robot.holonomic.runWithoutEncoder(0.0, 0.0, output)
                    hit()
                }
            }
        }

    }

    fun hit() {
        robot.ringLoadServo.position = ExtZoomBotConstants.RING_LOAD_SERVO_PUSH
        wait(ExtZoomBotConstants.AUTO_SHOOT_WAIT.toLong()) {updateVelocityToTelemetry()}
        robot.ringLoadServo.position = ExtZoomBotConstants.RING_LOAD_SERVO_BACK
        wait(ExtZoomBotConstants.AUTO_SHOOT_WAIT.toLong()) {updateVelocityToTelemetry()}
    }

    /**
     * Creates and operates [ReflectiveTelemetryMenu] before the init period.
     * Controls code until [isStopRequested] or [isStarted] is true.
     */

    fun updateVelocityToTelemetry() {
        telemetry.addData("vel", robot.zoomWheel.velocity)
        telemetry.update()
    }

    fun wait(timeMs: Long, and: ()->Unit) {
        val start = System.currentTimeMillis()
        while (System.currentTimeMillis() < start + timeMs) and()
    }

    fun operateMenu() {

        val dpadUpWatch = ToggleButtonWatcher {gamepad1.dpad_up}
        val dpadDownWatch = ToggleButtonWatcher {gamepad1.dpad_down}
        val dpadLeftWatch = ToggleButtonWatcher {gamepad1.dpad_left}
        val dpadRightWatch = ToggleButtonWatcher {gamepad1.dpad_right}

        config.update()

        while (!isStarted && !isStopRequested) {
            when {
                dpadUpWatch.invoke()    -> config.update(prevItem = true)
                dpadDownWatch.invoke()  -> config.update(nextItem = true)
                dpadLeftWatch.invoke()  -> config.update(iterBack = true)
                dpadRightWatch.invoke() -> config.update(iterFw   = true)
            }

            when {
                gamepad1.x -> robot.wobbleGrabber.move(grab = WobbleGrabber.GrabPosition.GRAB)
                gamepad1.y -> {
                    robot.wobbleGrabber.move(pivot = WobbleGrabber.PivotPosition.YEET)
                    robot.wobbleGrabber.move(grab = WobbleGrabber.GrabPosition.MID_GRAB)
                }
            }
        }

    }


    /**
     * Robot strafes for desired distance until motors have reached target.
     * Forwards distances and power to [robot.holonomic]
     */
    private fun drive(x: Double, y: Double, power: Double) {
        robot.holonomic.runUsingEncoder(x, y, power)
        val originalRuntime = runtime
        @Suppress("StatementWithEmptyBody") while (opModeIsActive() && robot.holonomic.motorsAreBusy() && runtime - originalRuntime < 3) robot.holonomicRR.update();
        robot.holonomic.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER)
    }

    /**
     * Robot strafes with specified power until desired time length is reached.
     * Forwards power to [robot.holonomic]
     */
    private fun timeDrive(x: Double, y: Double, z: Double, timeMs: Long) {
        val startingRuntime = System.currentTimeMillis()
        robot.holonomic.runWithoutEncoder(x, y, z)
        while (System.currentTimeMillis() - startingRuntime < timeMs) robot.holonomicRR.update()
        robot.holonomic.stop()
        robot.holonomic.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER)

    }

    private fun forDurationMs(duration: Long, action: ()->Unit) {
        val end = System.currentTimeMillis() + duration
        while (System.currentTimeMillis() < end) action.invoke()
    }

    /**
     * Reverses input number if [testColor] matches [allianceColor]
     */
    private infix fun Double.reverseIf(testColor: AllianceColor) : Double =
            if (this@PowerSeekTest.allianceColor==testColor) -this else this

    /**
     * Reverses input number if [testLine] matches [startingLine]
     */
    private infix fun Double.reverseIf(testLine: StartingLine): Double =
            if (this@PowerSeekTest.startingLine==testLine) -this else this

    private fun builder() = robot.holonomicRR.trajectoryBuilder()
    private fun builder(tangent: Double) = robot.holonomicRR.trajectoryBuilder(tangent)

    private fun BaseTrajectoryBuilder<TrajectoryBuilder>.buildAndRun(vararg waypointActions: Pair<Double, ()->Unit>) =
            robot.holonomicRR.followTrajectorySync(this.build(), waypointActions.toList())

}