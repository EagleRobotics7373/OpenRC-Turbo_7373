package org.firstinspires.ftc.teamcode.opmodes.gen1

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.BLUE
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor.RED
import org.firstinspires.ftc.teamcode.library.functions.Position.*
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtMisumiRobot
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.DriveConstantsBlueMisumi
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.DriveConstantsSlowMisumi
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.DriveConstantsTunedMisumi
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.AutoBlockIntake
import org.firstinspires.ftc.teamcode.library.vision.skystone.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.skystone.opencv.OpenCvContainer
import org.firstinspires.ftc.teamcode.library.vision.skystone.opencv.PixelStatsPipeline
import org.firstinspires.ftc.teamcode.opmodes.gen2.AutonomousConstants.*
import java.util.*
import kotlin.math.PI

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous State (Kotlin + RR)", group = "Main")
class AutonomousState_RR : LinearOpMode() {

    /*
        VARIABLES: Hardware and Control
     */
    private lateinit var robot           : ExtMisumiRobot
    private lateinit var imuController   : IMUController

    private          val telem           : MultipleTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    private lateinit var elapsedTime     : ElapsedTime

    private lateinit var cvContainer     : OpenCvContainer<PixelStatsPipeline>

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
    var visionDetector                      = PixelStatsPipeline.StatsDetector.DETECTOR_VALUE_STDDEV
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
        robot = ExtMisumiRobot(hardwareMap)
        imuController = robot.imuControllerA
        cvContainer = VisionFactory.createOpenCv(
                VisionFactory.CameraType.WEBCAM,
                hardwareMap,
                PixelStatsPipeline(PixelStatsPipeline.StatsDetector.DETECTOR_VALUE_STDDEV))


        robot.autoBlockIntakeRear.pivotIn18()
        robot.autoBlockIntakeRear.releaseBlock()

        robot.autoBlockIntakeFront.pivotIn18()
        robot.autoBlockIntakeFront.releaseBlock()

        robot.foundationGrabbersFront.unlock()


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
        if (liftLowerOnly) doIntakeDrop()
        else when (autonomousType) {
            AutonomousType.PARK -> doParkOnlyAuto();
            AutonomousType.FOUNDATION_BASIC -> doBasicFoundationAuto()
            AutonomousType.FOUNDATION_RR -> doRRFoundationAuto()
            AutonomousType.MULTISTONE -> doCrossField()
        }


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

    fun doParkOnlyAuto() {
        if (doLiftLower) doIntakeDrop()
        sleep(delayBeforeParking * 1000.toLong())
        drive(0.0, 12.0, 0.5)
    }

    fun doBasicFoundationAuto() {
        robot.foundationGrabbersFront.mid()
        drive(24.0, 0.0, 0.8)
        robot.foundationGrabbersFront.lock()
        sleep(700)
        timeDrive(0.0, -0.5, 0.0, 2000)
        robot.foundationGrabbersFront.unlock()
        sleep(700)
    }

    fun doRRFoundationAuto() {
        robot.foundationGrabbersFront.mid()
        robot.holonomicRR.poseEstimate = Pose2d(38.0, 63.0 reverseIf RED, (-90.0 reverseIf RED).toRadians())
        builder()
                .strafeTo(Vector2d(48.0, 48.0 reverseIf RED))
                .strafeTo(Vector2d(51.75, 30.5 reverseIf RED))
                .buildAndRun()

        robot.foundationGrabbersFront.lock()
        sleep(700)

        if (allianceColor == BLUE) {
            timeDrive(0.4, -0.7, -0.6, 1100)
            timeDrive(0.0, 0.8, -0.15, 1000)
        } else {
            timeDrive(-0.4, -0.75, 0.475, 1000)
            timeDrive(0.0, 0.8, 0.2, 1000)
        }

        robot.foundationGrabbersFront.unlock()
        sleep(700)

//                    DriveConstantsTunedMisumi.BASE_CONSTRAINTS.maxVel = 85.0
//                    DriveConstantsTunedMisumi.BASE_CONSTRAINTS.maxAccel = 70.0

        val parkLocation = (if (parkNearDS) 62.0 else 38.0) reverseIf RED

        robot.holonomicRR.trajectoryBuilder(DriveConstraints( //                    50.0, 30.0, 40.0,
                80.0, 70.0, 40.0,
                Math.PI, Math.PI, 0.0
        ))
                .strafeTo(Vector2d(0.0, parkLocation))
                .buildAndRun()


    }

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
                                      else Pose2d(-39.25, -62.0, PI)

        builder(PI)
                .splineToConstantHeading(Pose2d(stonePositionsFromWall[stonesOrder[0]], nextToStonePosY, PI/-2))
                .buildAndRun(Pair(0.30) { neededBlockGrabber.pivotMid(); neededBlockGrabber.grabberMid(); })
        doBlockGrab(neededBlockGrabber)

        builder()
                .strafeTo(Vector2d(-16.0, drivingAgainstBridgePosY))
                .buildAndRun()

        if (doLiftLower) doIntakeDrop()

        builder(0.0)
                .splineToConstantHeading(Pose2d(RR_PAST_BRIDGE_X, drivingAgainstBridgePosY))
                .splineToConstantHeading(Pose2d(foundationPlacementPositions[0], againstFoundationY))
                .buildAndRun(Pair(0.60) { neededBlockGrabber.pivotMid(); })

        for (i in stonesOrder.indices) {

            doBlockRelease(neededBlockGrabber)

            if (i < stonesOrder.size - 1) {
                builder(PI)
                        .splineToConstantHeading(Pose2d(18.0, drivingAgainstBridgePosY, PI))
                        .splineToConstantHeading(Pose2d(-16.0, drivingAgainstBridgePosY, PI))
                        .splineToConstantHeading(Pose2d(stonePositionsFromWall[stonesOrder[i + 1]], nextToStonePosY, PI))
                        .buildAndRun(Pair(0.60) { neededBlockGrabber.pivotMid(); neededBlockGrabber.grabberMid(); })

                doBlockGrab(neededBlockGrabber)

                builder(0.0)
                        .splineToConstantHeading(Pose2d(-16.0, drivingAgainstBridgePosY))
                        .splineToConstantHeading(Pose2d(RR_PAST_BRIDGE_X, drivingAgainstBridgePosY))
                        .splineToConstantHeading(Pose2d(foundationPlacementPositions[i], againstFoundationY))
                        .buildAndRun(Pair(0.40) { neededBlockGrabber.pivotMid(); })

            } else {
                neededBlockGrabber.releaseBlock()
                neededBlockGrabber.pivotUp()
                if (doFoundationPull) {

                    builder(PI)
                            .splineToConstantHeading(Pose2d(48.0, 45.0 reverseIf RED, 0.0))
                            .splineTo(Pose2d(52.00, 28.50, (-87.50).toRadians()))
                            .buildAndRun(
                                    Pair(0.50) {robot.foundationGrabbersFront.mid()},
                                    Pair(0.95) {robot.foundationGrabbersFront.lock()}
                            )

                    sleep(350)

                    if (allianceColor == BLUE) {
                        timeDrive(0.4, -0.7, -0.6, 1100)
                        timeDrive(0.0, 0.8, -0.15, 1000)
                    } else {
                        timeDrive(-0.4, -0.75, 0.475, 1100)
                        timeDrive(0.0, 0.85, 0.40, 1500)
                    }


                    robot.foundationGrabbersFront.unlock()

//                    DriveConstantsTunedMisumi.BASE_CONSTRAINTS.maxVel = 85.0
//                    DriveConstantsTunedMisumi.BASE_CONSTRAINTS.maxAccel = 70.0

                    robot.holonomicRR.trajectoryBuilder(DriveConstraints( //                    50.0, 30.0, 40.0,
                            100.0, 100.0, 40.0,
                            Math.PI, Math.PI, 0.0
                    ))
                            .strafeTo(Vector2d(0.0, 38.0 reverseIf RED))
                            .buildAndRun()

                } else {
                    builder()
                            .strafeTo(Vector2d(0.0, 37.5 reverseIf RED))
                            .buildAndRun()
                }
                break
            }
        }



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
            if (gamepad1.x && !doLiftLower) {
                robot.intakeLiftRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                doLiftLower = true
                robot.intakeLiftRight.targetPosition = -1100
                robot.intakeLiftRight.mode = DcMotor.RunMode.RUN_TO_POSITION
                robot.intakeLiftRight.power = 0.3
                while (gamepad1.x && !isStarted);
//                robot.intakeLiftRight.power = 0.0
//                robot.intakeLiftRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
//                robot.intakeLiftRight.power = 0.0
                menu.refresh()
            }
        }


    }

    private val neededBlockGrabber : AutoBlockIntake
        get() = if (allianceColor == RED) robot.autoBlockIntakeFront else robot.autoBlockIntakeRear

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

    private fun doIntakeDrop() {
//        while (opModeIsActive()) {
//            val intakeLiftCurrent = robot.intakeLiftRight.currentPosition
//            val inputSpoolPower =
//                        if (intakeLiftCurrent > 0.0) 0.0
//                        else if (intakeLiftCurrent + 75 > 0.0) 0.20
//                        else if (intakeLiftCurrent + 500 > 0.0) 0.45
//                        else if (intakeLiftCurrent + 800 > 0.0) 0.45
//                        else if (intakeLiftCurrent + 1200 > 0.0) 0.60
//                        else if (intakeLiftCurrent + 2000 > 0.0) 0.75
//                        else 1.0
//
//            robot.intakeLiftLeft.power = -inputSpoolPower
//            robot.intakeLiftRight.power = inputSpoolPower
//
//            if (intakeLiftCurrent > -316) {
//                robot.intakeLiftLeft.power = 0.0
//                robot.intakeLiftRight.power = 0.0
//                break
//            }
//        }
        robot.intakeLiftRight.mode = DcMotor.RunMode.RUN_TO_POSITION
        do {
            robot.intakeLiftRight.targetPosition = INTAKE_DROP_POSITION
            telemetry.addData("current", robot.intakeLiftRight.currentPosition)
            telemetry.update()
            robot.intakeLiftRight.power = 1.0
            robot.intakeLiftRight.targetPositionTolerance = 20
        }
        while (opModeIsActive() and robot.intakeLiftRight.isBusy)
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
    private fun builder(tangent: Double) = robot.holonomicRR.trajectoryBuilder(tangent)

    private fun BaseTrajectoryBuilder<TrajectoryBuilder>.buildAndRun(vararg waypointActions: Pair<Double, ()->Unit>) =
            robot.holonomicRR.followTrajectorySync(this.build(), waypointActions.toList())

}