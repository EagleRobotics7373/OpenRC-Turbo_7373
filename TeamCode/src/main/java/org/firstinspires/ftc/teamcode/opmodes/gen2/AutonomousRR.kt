package org.firstinspires.ftc.teamcode.opmodes.gen1

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.*
import org.firstinspires.ftc.teamcode.library.functions.StartingLine.*
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtRingPlaceBot
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.WobbleGrabber
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.base.OpenCvContainer
import org.firstinspires.ftc.teamcode.library.vision.ultimategoal.RingPixelAnalysisPipeline
import org.firstinspires.ftc.teamcode.library.vision.ultimategoal.UltimateGoalVisionConstants
import kotlin.math.PI
import org.firstinspires.ftc.teamcode.opmodes.gen2.OpModeConfig

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous State (Kotlin + RR)", group = "Main")
class AutonomousRR : LinearOpMode() {

    /*
        VARIABLES: Hardware and Control
     */
    private lateinit var robot           : ExtRingPlaceBot
    private lateinit var imuController   : IMUController

    private          val telem           : MultipleTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    private lateinit var elapsedTime     : ElapsedTime

    private lateinit var cvContainer     : OpenCvContainer<RingPixelAnalysisPipeline>

    private lateinit var player          : ExtDirMusicPlayer

    /*
        VARIABLES: Menu Options
     */
    private val config = OpModeConfig(telemetry)
    private var allianceColor: AllianceColor by config.custom("Alliance Color", RED, BLUE)
    private var startingLine: StartingLine by config.custom("Starting Line", CENTER, FAR)

    override fun runOpMode() {
        /*
            Main autonomous variable initialization
         */
        robot = ExtRingPlaceBot(hardwareMap)
        imuController = robot.imuControllerC
        cvContainer = VisionFactory.createOpenCv(
                VisionFactory.CameraType.WEBCAM,
                hardwareMap,
                RingPixelAnalysisPipeline())


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


        elapsedTime = ElapsedTime()



        /*
            Perform actions
         */
        cvContainer.pipeline.shouldKeepTracking = false

        if (allianceColor == RED && startingLine == FAR || allianceColor == BLUE && startingLine == CENTER) {
            UltimateGoalVisionConstants.BOTTOM_RING_CENTER_X = UltimateGoalVisionConstants.BOTTOM_RING_CENTER_X_ON_LEFT
            UltimateGoalVisionConstants.BOTTOM_RING_CENTER_Y = UltimateGoalVisionConstants.BOTTOM_RING_CENTER_Y_ON_LEFT
        } else {
            UltimateGoalVisionConstants.BOTTOM_RING_CENTER_X = UltimateGoalVisionConstants.BOTTOM_RING_CENTER_X_ON_RIGHT
            UltimateGoalVisionConstants.BOTTOM_RING_CENTER_Y = UltimateGoalVisionConstants.BOTTOM_RING_CENTER_Y_ON_RIGHT
        }

        cvContainer.pipeline.tracking = true
        while (cvContainer.pipeline.tracking);

        doFullAuto()

        /*
            OpMode actions have finished. Wait until OpMode is stopped, then close resources.
         */
        while (opModeIsActive()) robot.holonomicRR.update()

        Thread {
            cvContainer.stop()
            cvContainer.camera.closeCameraDevice()
        }.start()

    }

    fun doFullAuto() {

        // Get the number of viewable rings from OpenCV
        val numRings = cvContainer.pipeline.numberOfRings

        // Create a static map of wobble goal drop-off positions for each number of rings
        val wobbleDropoffs = mapOf(
                0 to Vector2d(-6.5, -60.0 reverseIf BLUE),
                1 to Vector2d(16.0, -36.0 reverseIf BLUE),
                4 to Vector2d(40.0, -60.0 reverseIf BLUE)
        )

        // Set the robot starting position within RoadRunner
        robot.holonomicRR.poseEstimate = Pose2d(
                -63.0,
                (if (startingLine == CENTER) -24.0 else -48.0) reverseIf BLUE,
                0.0
        )

        // Do the initial movement to wobble drop-off location
        builder(PI.div(4) reverseIf BLUE reverseIf FAR)
                // Spline to drive around and preserve the starter stack
                .splineToConstantHeading(
                        endPosition = Vector2d(
                                x = -24.0,
                                y = robot.holonomicRR.poseEstimate.y.plus(6.0 reverseIf FAR reverseIf BLUE)
                        ),
                        endTangent = 0.0
                )
                // Spline to the wobble drop-off position
                .splineToConstantHeading(
                        endPosition = wobbleDropoffs[numRings] ?: error("Incorrect number of rings set"),
                        endTangent = if (startingLine == CENTER) -PI.div(2) reverseIf BLUE else 0.0
                )
                .buildAndRun()

        robot.wobbleGrabber.pivot(WobbleGrabber.PivotPosition.PERPENDICULAR)
        sleep(1000)
        robot.wobbleGrabber.grab(WobbleGrabber.GrabPosition.STORAGE)
        sleep(500)
        robot.wobbleGrabber.pivot(WobbleGrabber.PivotPosition.STORAGE)
        sleep(1000)



        // Do subsequent movement to the ring drop-off
        builder(5*PI/4 reverseIf RED)
                // Motion to avoid the placed wobble goal. This will be different based on whether wobble is in pos A/C or B
                .run {
                    if (numRings == 1)
                        splineToConstantHeading(Vector2d(24.0, 13.0 reverseIf RED), -PI/4)
                                .splineToConstantHeading(Vector2d(48.0, 13.0 reverseIf RED), PI/4)
                    else
                        splineToConstantHeading(
                                Vector2d(
                                        x = robot.holonomicRR.poseEstimate.x,
                                        y = 37.0 reverseIf RED ),
                                endTangent = (-PI/4) reverseIf RED)
                }
                // Spline to ring drop-off
                .splineToConstantHeading(Vector2d(63.0, 37.0 reverseIf RED), 0.0)
                .buildAndRun()

        sleep(2000)

        // Move to parking line
        builder(PI/2 reverseIf BLUE)
                // Single spline onto the starting line, closest to center while still on alliance side
                .splineToConstantHeading(Vector2d(12.0, 12.0 reverseIf RED), -PI)
                .buildAndRun()


    }

    /**
     * Creates and operates [ReflectiveTelemetryMenu] before the init period.
     * Controls code until [isStopRequested] or [isStarted] is true.
     */

    fun operateMenu() {

        val dpadUpWatch = ToggleButtonWatcher {gamepad1.dpad_up}
        val dpadDownWatch = ToggleButtonWatcher {gamepad1.dpad_down}
        val dpadLeftWatch = ToggleButtonWatcher {gamepad1.dpad_left}
        val dpadRightWatch = ToggleButtonWatcher {gamepad1.dpad_right}

        config.update()

        while (!isStarted && !isStopRequested) {
            when {
                dpadUpWatch.call()    -> config.update(prevItem = true)
                dpadDownWatch.call()  -> config.update(nextItem = true)
                dpadLeftWatch.call()  -> config.update(iterBack = true)
                dpadRightWatch.call() -> config.update(iterFw   = true)
            }

            when {
                gamepad1.x -> robot.wobbleGrabber.grab(WobbleGrabber.GrabPosition.GRAB)
                gamepad1.y -> {
                    robot.wobbleGrabber.pivot(WobbleGrabber.PivotPosition.STORAGE)
                    robot.wobbleGrabber.grab(WobbleGrabber.GrabPosition.MID_GRAB)
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

    /**
     * Reverses input number if [testColor] matches [allianceColor]
     */
    private infix fun Double.reverseIf(testColor: AllianceColor) : Double =
            if (this@AutonomousRR.allianceColor==allianceColor) -this else this

    /**
     * Reverses input number if [testLine] matches [startingLine]
     */
    private infix fun Double.reverseIf(testLine: StartingLine): Double =
            if (this@AutonomousRR.startingLine==testLine) -this else this

    private fun builder() = robot.holonomicRR.trajectoryBuilder()
    private fun builder(tangent: Double) = robot.holonomicRR.trajectoryBuilder(tangent)

    private fun BaseTrajectoryBuilder<TrajectoryBuilder>.buildAndRun(vararg waypointActions: Pair<Double, ()->Unit>) =
            robot.holonomicRR.followTrajectorySync(this.build(), waypointActions.toList())

}