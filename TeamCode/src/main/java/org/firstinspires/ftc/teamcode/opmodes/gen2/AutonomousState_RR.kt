package org.firstinspires.ftc.teamcode.opmodes.gen1

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.BLUE
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.RED
import org.firstinspires.ftc.teamcode.library.functions.Position.*
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtRingPlaceBot
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.base.OpenCvContainer
import org.firstinspires.ftc.teamcode.library.vision.skystone.SkystonePixelStatsPipeline
import org.firstinspires.ftc.teamcode.opmodes.gen2.AutonomousConstants.*
import org.firstinspires.ftc.teamcode.opmodes.gen2.OpModeConfig

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous State (Kotlin + RR)", group = "Main")
class AutonomousState_RR : LinearOpMode() {

    /*
        VARIABLES: Hardware and Control
     */
    private lateinit var robot           : ExtRingPlaceBot
    private lateinit var imuController   : IMUController

    private          val telem           : MultipleTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    private lateinit var elapsedTime     : ElapsedTime

    private lateinit var cvContainer     : OpenCvContainer<SkystonePixelStatsPipeline>

    private lateinit var player          : ExtDirMusicPlayer

    /*
        VARIABLES: Menu Options
     */
    val config = OpModeConfig(telemetry)
    var allianceColor: AllianceColor by config.custom("Alliance Color", RED, BLUE)

    override fun runOpMode() {
        /*
            Main autonomous variable initialization
         */
        robot = ExtRingPlaceBot(hardwareMap)
        imuController = robot.imuControllerC
        cvContainer = VisionFactory.createOpenCv(
                VisionFactory.CameraType.WEBCAM,
                hardwareMap,
                SkystonePixelStatsPipeline(SkystonePixelStatsPipeline.StatsDetector.DETECTOR_VALUE_STDDEV))


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



        /*
            OpMode actions have finished. Wait until OpMode is stopped, then close resources.
         */
        while (opModeIsActive()) robot.holonomicRR.update()

        Thread {
            cvContainer.stop()
            cvContainer.camera.closeCameraDevice()
        }.start()

    }

//    override fun internalPostLoop() {
//        super.internalPostLoop()
//        player.stop()
//    }

    fun doCrossField() {
        cvContainer.pipeline.tracking = true
        while (cvContainer.pipeline.tracking);

        val skystonePosition = cvContainer.pipeline.skystonePos

        val skystonePositionFromWall =
                when (allianceColor) {
                    RED ->
                        when (skystonePosition) {
                            NULL  -> 2 // Not in camera frame
                            LEFT  -> 0 // LEFT
                            else  -> 1 // RIGHT, or unknown
                        }
                    BLUE ->
                        when (skystonePosition) {
                            RIGHT -> 0 // RIGHT
                            LEFT  -> 1 // LEFT
                            else  -> 2 // Not in camera frame, or unknown
                        }
                }

        val stonesOrder = /*arrayOf(STONE_0, STONE_1, STONE_2, STONE_3).take(numStonesToMove)*/
            when (skystonePositionFromWall) {
                0    -> arrayOf(0, 3, 5, 4)
                1    -> arrayOf(1, 4, 5, 3)
                else -> arrayOf(2, 5, 4, 3)
            }.take(numStonesToMove)

        telemetry.addData("Skystone pos from wall", skystonePositionFromWall)
        telemetry.update()

        val nextToStonePosY = RR_NEXT_TO_STONE_Y reverseIf RED
        val drivingAgainstBridgePosY = RR_AGAINST_BRIDGE_Y reverseIf RED
        val againstFoundationY = RR_AGAINST_FOUNDATION_Y reverseIf RED

        val stonePositionsFromWall =
                (if (allianceColor == BLUE) arrayOf(-62.0, -55.0, -48.0, -42.0, -31.0, -23.0)
                        else arrayOf(-64.0, -55.0, -48.0, -39.0, -31.0, -23.0))

                        .map { if (allianceColor == RED) it + RR_RED_STONE_OFFSET else it}

        val foundationPlacementPositions = arrayOf(
                42.0, 46.0, 52.0, 52.0).map { if (allianceColor == RED) it + RR_RED_STONE_OFFSET else it}

        robot.holonomicRR.poseEstimate =
                if (allianceColor == BLUE) Pose2d(-38.5, 63.0, 0.0)
                                      else Pose2d(-39.25, -62.0, 180.0.toRadians())


        builder()
                .strafeTo(Vector2d(-16.0, drivingAgainstBridgePosY))
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
    private infix fun Double.reverseIf(testColor: AllianceColor) : Double {
        return this * if (allianceColor==testColor) -1.0 else 1.0
    }

    private fun builder() = robot.holonomicRR.trajectoryBuilder()

    private fun BaseTrajectoryBuilder.buildAndRun(vararg waypointActions: Pair<Double, ()->Unit>) =
            robot.holonomicRR.followTrajectorySync(this.build(), waypointActions.toList())

}