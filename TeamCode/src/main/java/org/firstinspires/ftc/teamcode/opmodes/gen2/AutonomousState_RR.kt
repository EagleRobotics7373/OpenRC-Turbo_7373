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
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.DriveConstantsBlueMisumi
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.DriveConstantsSlowMisumi
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.DriveConstantsTunedMisumi
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.AutoBlockIntake
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.base.OpenCvContainer
import org.firstinspires.ftc.teamcode.library.vision.skystone.SkystonePixelStatsPipeline
import org.firstinspires.ftc.teamcode.opmodes.gen2.AutonomousConstants.*

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
    var allianceColor                       = _allianceColor
//    var startingPosition                    = _startingPosition
    var autonomousType               = _autonomousType
    var driveClass                              = _driveClass
    var musicFile                           = _musicFile
    var parkOnly                            = _parkOnly
    var visionDetector                      = SkystonePixelStatsPipeline.StatsDetector.DETECTOR_VALUE_STDDEV
    var delayBeforeParking                  = _delayBeforeParking
//    var foundationSwivel                    = _foundationSwivel
    var doFoundationPull                    = _doFoundationPull
    var doParkAtEnd                                 = _doParkAtEnd
    var liftLowerOnly = _liftLowerOnly
    var doLiftLower = false
    var parkNearDS                          = false
    var numStonesToMove                    = _numStonesToMove

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
        RobotConstantsAccessor.loadedOdometryClass = driveClass
        robot.holonomicRR.redefine()

        player = ExtDirMusicPlayer(musicFile, true)
        player.play()

        elapsedTime = ElapsedTime()

        cvContainer.pipeline.detector = visionDetector


        /*
            Perform actions
         */



        /*
            OpMode actions have finished. Wait until OpMode is stopped, then close resources.
         */
        while (opModeIsActive()) robot.holonomicRR.update()

        player.stop()
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
        val menu = ReflectiveTelemetryMenu(telem,
                ReflectiveMenuItemFeedback("Status") { if (doLiftLower) "Prepared" else "NOT PREPARED!"},
                ReflectiveMenuItemEnum("Alliance Color", ::allianceColor, AllianceColor.RED, AllianceColor.BLUE),
                ReflectiveMenuItemEnum("Autonomous Type", ::autonomousType, *AutonomousType.values()),
                ReflectiveMenuItemEnum("Constants", ::driveClass, DriveConstantsTunedMisumi::class.java, DriveConstantsSlowMisumi::class.java, DriveConstantsBlueMisumi::class.java, toStringMethod = { it.simpleName }),
                ReflectiveMenuItemEnum("Music", ::musicFile, *ExtMusicFile.values()),
//                ReflectiveMenuItemBoolean("Foundation Swivel", ::foundationSwivel),
                ReflectiveMenuItemBoolean("Foundation Pull", ::doFoundationPull),
                ReflectiveMenuItemBoolean("Park at End", ::doParkAtEnd),
                ReflectiveMenuItemBoolean("Lift Lower Only", ::liftLowerOnly),
                ReflectiveMenuItemInteger("Number of Stones", ::numStonesToMove, 1, 3, 1),
                ReflectiveMenuItemInteger("Park Delay", ::delayBeforeParking, 0, 25, 1),
                ReflectiveMenuItemBoolean("Park Near DS (FOUNDATION TYPE)", ::parkNearDS)
        )

        val dpadUpWatch = ToggleButtonWatcher {gamepad1.dpad_up}
        val dpadDownWatch = ToggleButtonWatcher {gamepad1.dpad_down}
        val dpadLeftWatch = ToggleButtonWatcher {gamepad1.dpad_left}
        val dpadRightWatch = ToggleButtonWatcher {gamepad1.dpad_right}

        while (!isStarted && !isStopRequested) {
            when {
                dpadUpWatch.call()    -> menu.previousItem()
                dpadDownWatch.call()  -> menu.nextItem()
                dpadLeftWatch.call()  -> menu.iterateBackward()
                dpadRightWatch.call() -> menu.iterateForward()
            }
        }


    }

    private fun doBlockGrab(grabber: AutoBlockIntake) {
        grabber.pivotDown()
        sleep(600)
        grabber.grabBlock()
        sleep(750)
        grabber.pivotUp()
        sleep(400)
    }

    private fun doBlockRelease(grabber: AutoBlockIntake) {
        grabber.releaseBlock()
        sleep(350)
        grabber.pivotUp()
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

    fun imuPIRotate(angle: Double) {
        var currentValue = imuController.getHeading().toDegrees()
        val targetValue = currentValue + angle

        val Kp = .02 // Proportional Constant
        val Ki = .0007 // Integral Constant
        var et: Double // Error
        var proportionPower: Double
        var integralPower: Double
        var power: Double
        var errorSum = 0.0
        val originalRuntime = runtime
        while (currentValue != targetValue && opModeIsActive() && runtime - originalRuntime < 4) {
            currentValue = imuController.getHeading().toDegrees()
            telemetry.addData("Current value", currentValue)
            telemetry.addData("Target value", targetValue)

            if (currentValue < 0) {
                et = -(Math.abs(targetValue) - Math.abs(currentValue))
            } else {
                et = targetValue - currentValue
            }


            if (Kp * et > .8) {
                proportionPower = .8
            } else {
                proportionPower = Kp * et
            }

            if (Math.abs(et) < 45) {
                errorSum += et
            }

            integralPower = Ki * errorSum

            power = -(proportionPower + integralPower)
            telemetry.addData("et", et)
            telemetry.addData("propPw", proportionPower)
            telemetry.addData("intPw", integralPower)
            telemetry.addData("errorsum", errorSum)
            telemetry.addData("Power", power)
            robot.holonomic.runWithoutEncoder(0.0, 0.0, power * 0.30)
            telemetry.update()
        }
        robot.holonomic.stop()
    }

    private fun builder() = robot.holonomicRR.trajectoryBuilder()

    private fun BaseTrajectoryBuilder.buildAndRun(vararg waypointActions: Pair<Double, ()->Unit>) =
            robot.holonomicRR.followTrajectorySync(this.build(), waypointActions.toList())

}