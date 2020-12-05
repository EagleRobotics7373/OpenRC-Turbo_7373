package org.firstinspires.ftc.teamcode.opmodes.gen3

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtZoomBot
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtZoomBotConstants
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.RingTapper
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.WobbleGrabber
import org.firstinspires.ftc.teamcode.library.vision.base.OpenCvContainer
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory
import org.firstinspires.ftc.teamcode.library.vision.ultimategoal.IntakeRingViewingPipeline
import kotlin.math.absoluteValue

@TeleOp(name="TeleOp RD Gen3", group="Gen3 Basic")
open class TeleOpRD : OpMode() {
    // robot hardware declaration; assignment occures during init()
    lateinit var robot : ExtZoomBot

    // gamepad toggle button watchers, instantiated after opmode init
    private lateinit var watch_gamepad1_buttonY : ToggleButtonWatcher
    protected lateinit var watch_gamepad1_dpadDown : ToggleButtonWatcher
    protected lateinit var watch_gamepad1_dpadUp : ToggleButtonWatcher
    protected lateinit var watch_gamepad1_dpadLeft : ToggleButtonWatcher
    protected lateinit var watch_gamepad1_dpadRight : ToggleButtonWatcher
    private lateinit var watch_gamepad2_leftStickButton : ToggleButtonWatcher
    private lateinit var watch_gamepad2_rightStickButton : ToggleButtonWatcher
    private lateinit var watch_gamepad1_buttonA : ToggleButtonWatcher
    private lateinit var watch_gamepad1_buttonB : ToggleButtonWatcher
    private lateinit var watch_gamepad2_buttonA : ToggleButtonWatcher
    private lateinit var watch_gamepad2_buttonB : ToggleButtonWatcher
    private lateinit var watch_gamepad2_dpadLeft : ToggleButtonWatcher
    private lateinit var watch_gamepad2_dpadRight : ToggleButtonWatcher
    private lateinit var watch_gamepad2_dpadUp : ToggleButtonWatcher
    private lateinit var watch_gamepad2_dpadDown : ToggleButtonWatcher
    private lateinit var watch_gamepad1_leftBumper : ToggleButtonWatcher
    private lateinit var watch_gamepad1_rightBumper : ToggleButtonWatcher

//    val musicPlayer = ExtDirMusicPlayer(ExtMusicFile.TETRIS)
    var playingMusic = false
    var elapsedTime : ElapsedTime? = null
    var last = 0.0
    var current = 0.0
    var reverse = false
    var speed = 3
    var lastCycleTime = System.currentTimeMillis()
    lateinit var originalPID: PIDFCoefficients

    protected val canControlDrivetrainOrientation: Boolean
    get() = (!gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad1.start)

    protected val gamepad1CanControlIntakeOrientation: Boolean
        get() = (gamepad1.left_bumper && gamepad1.right_bumper && !gamepad1.start)

    protected val gamepad2CanSetVelocityConstants: Boolean
        get() = (gamepad2.left_bumper && gamepad2.right_bumper)

    var intakeLiftBaseline = 0
    var intakePivotBaseline = 0
    var reverseIntakeLift = true
    var useLEDs             = true

    var container : OpenCvContainer<IntakeRingViewingPipeline>? = null

    override fun init() {
        // instantiate robot variables
        robot = ExtZoomBot(hardwareMap)
        robot.expansionhubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL}
        // Instantiate toggle button watchers. Each statement below is calling a constructor with a single
        //   parameter, in this case being a function for calling the gamepad button. This does not set the
        //   current gamepad state only in the ToggleButtonWatcher; rather, it sets the ability for the
        //   ToggleButtonWatcher to invoke the gamepad values.
        watch_gamepad1_buttonY = ToggleButtonWatcher {gamepad1.y}
        watch_gamepad1_dpadDown = ToggleButtonWatcher {gamepad1.dpad_down}
        watch_gamepad1_dpadUp = ToggleButtonWatcher {gamepad1.dpad_up}
        watch_gamepad1_dpadLeft = ToggleButtonWatcher {gamepad1.dpad_left}
        watch_gamepad1_dpadRight = ToggleButtonWatcher {gamepad1.dpad_right}
        watch_gamepad1_buttonA = ToggleButtonWatcher { gamepad1.a }
        watch_gamepad1_buttonB = ToggleButtonWatcher { gamepad1.b }
        watch_gamepad2_leftStickButton = ToggleButtonWatcher { gamepad2.left_stick_button }
        watch_gamepad2_rightStickButton = ToggleButtonWatcher { gamepad2.right_stick_button }
        watch_gamepad2_buttonA = ToggleButtonWatcher { gamepad2.a }
        watch_gamepad2_buttonB = ToggleButtonWatcher { gamepad2.b && !gamepad2.start }
        watch_gamepad2_dpadLeft = ToggleButtonWatcher { gamepad2.dpad_left }
        watch_gamepad2_dpadRight = ToggleButtonWatcher { gamepad2.dpad_right }
        watch_gamepad2_dpadUp = ToggleButtonWatcher { gamepad2.dpad_up }
        watch_gamepad2_dpadDown = ToggleButtonWatcher { gamepad2.dpad_down }
        watch_gamepad1_leftBumper = ToggleButtonWatcher { gamepad1.left_bumper }
        watch_gamepad1_rightBumper = ToggleButtonWatcher { gamepad1.right_bumper }

        intakeLiftBaseline = 0
        this.telemetry = MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry)
        container = VisionFactory.createOpenCv(VisionFactory.CameraType.WEBCAM, hardwareMap, IntakeRingViewingPipeline())
        container?.pipeline?.shouldKeepTracking = true
        originalPID = robot.zoomWheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER)
    }

    override fun start() {
        container?.pipeline?.tracking = true
        robot.wobbleGrabber.state = WobbleGrabber.WobbleGrabberState.STORAGE
    }

    override fun loop() {
        // invoke methods for each individual system for streamlined navigation
        robot.expansionhubs.forEach { it.clearBulkCache() }
        controlDrivetrain()
        controlOtherDevices()
        controlIntakeMechanism()
        controlTelemetry()
        controlMusic()
//        robot.holonomicRR.update()
//        if (!useLEDs) robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK)
    }

    override fun stop() {
        // overrides OpMode.stop() to ensure hardware components and music player stop
        super.stop()
        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK)
//        thread { container.stop() }
//        musicPlayer.stop()
    }

    protected open fun controlDrivetrain() {
        // Set x, y, and z inputs:
        //    Use Double.rangeBuffer to set motor power dead-band
        //    Use Double.times() for variable speed. (This functional method works the same as doing x * y in Java)
        //    Use Double.reverseIf() on x and y to easily reverse input direction
        val x = gamepad1.left_stick_x .toDouble().rangeBuffer(-0.1, 0.1, 0.0).times(0.33*speed).reverseIf(reverse)
        val y = -gamepad1.left_stick_y.toDouble().rangeBuffer(-0.1, 0.1, 0.0).times(0.33*speed).reverseIf(reverse)
        val z = gamepad1.right_stick_x.toDouble().rangeBuffer(-0.1, 0.1, 0.0).times(0.33*speed)

        // set gamepad inputs to robot output
        robot.holonomicRR.runWithoutEncoder(x, y, z)

        // read 'a' and 'b' buttons to reverse robot orientation
        if (canControlDrivetrainOrientation && gamepad1.a) reverse = false
        else if (canControlDrivetrainOrientation && gamepad1.b) reverse = true

        // reverse robot orientation using toggle feature in addition to static buttons
        // if y button is pressed, make reverse variable opposite of what it is now
        //   then wait until y is released before letting it change the variable again
        if (canControlDrivetrainOrientation && watch_gamepad1_buttonY.invoke()) reverse = !reverse

        // increase or decrease robot speed variable using toggle system
        // speed variable should be between 1 and 3
        //    3 = max power (0.33 * 3 * input)
        //    2 = mid power (0.33 * 2 * input)
        //    1 = low power (0.33 * 1 * input)
        if (watch_gamepad1_dpadDown.invoke() and (speed > 1)) speed--
        if (watch_gamepad1_dpadUp.invoke() and (speed < 3)) speed++


    }

    /**
     * Control wobble grabber
     */
    private fun controlOtherDevices() {
//        when {
//            gamepad2.y -> robot.wobbleGrabber.releaseGrab()     // y is for yeet
//            gamepad2.x -> robot.wobbleGrabber.grab()
//            gamepad2.b -> robot.wobbleGrabber.midGrab()
//        }

//        if (gamepad2.left_bumper) {
//            // Let's provide options for grabbing the intake
//            when {
//                gamepad2.a -> robot.wobbleGrabber.move(pivot = WobbleGrabber.PivotPosition.GRAB)
//                gamepad2.x -> robot.wobbleGrabber.move(grab = WobbleGrabber.GrabPosition.GRAB)
//                gamepad2.b -> robot.wobbleGrabber.move(pivot = WobbleGrabber.PivotPosition.PERPENDICULAR)
//                gamepad2.y -> robot.wobbleGrabber.move(grab = WobbleGrabber.GrabPosition.MID_GRAB)
//            }
//        } else if (gamepad2.right_bumper) {
//            // Let's provide options for releasing the intake in yeet position
//            when {
//                gamepad2.a -> robot.wobbleGrabber.move(pivot = WobbleGrabber.PivotPosition.YEET)
//                gamepad2.x -> robot.wobbleGrabber.move(pivot = WobbleGrabber.PivotPosition.OVER_WALL)
//                gamepad2.b -> robot.wobbleGrabber.move(pivot = WobbleGrabber.PivotPosition.STORAGE)
//                gamepad2.y -> robot.wobbleGrabber.move(grab = WobbleGrabber.GrabPosition.STORAGE)
//            }
//        }
//
        when {
            (watch_gamepad2_leftStickButton()) || (watch_gamepad1_leftBumper() && gamepad1.start) -> robot.wobbleGrabber.prevState()
            (watch_gamepad2_rightStickButton()) || (watch_gamepad1_rightBumper() && gamepad1.start) -> robot.wobbleGrabber.nextState()
        }
        robot.wobbleGrabber.update()



    }

    // set PID control coefficients for auto-intake raise in controlIntakeMechanism()
    // this should eventually be moved

//    var controlledZoomSpeed = 0.5
    private fun controlIntakeMechanism() {
        val power = when {
            gamepad2.left_trigger > 0.05 -> -gamepad2.left_trigger.toDouble()
            gamepad2.right_trigger > 0.05 -> gamepad2.right_trigger.toDouble()
            gamepad1.left_trigger > 0.05 -> -gamepad1.left_trigger.toDouble()
            gamepad1.right_trigger > 0.05 -> gamepad1.right_trigger.toDouble()
            else -> 0.0
        }
        robot.intakeStage1.power = if (gamepad2.left_stick_y.absoluteValue > 0.05) gamepad2.left_stick_y.toDouble() else power
        robot.intakeStage2.power = if (gamepad2.right_stick_y.absoluteValue > 0.05) gamepad2.right_stick_y.toDouble() else power

//        robot.intakeStage1.power = gamepad2.left_stick_y.toDouble();
//        robot.intakeStage2.power = gamepad2.right_stick_y.toDouble();

        when {
            (gamepad2.a && !gamepad2CanSetVelocityConstants) || (this.gamepad1CanControlIntakeOrientation && gamepad1.a) ->
                ExtZoomBotConstants.ACTIVE_FLYWHEEL = true
            (gamepad2.b && !gamepad2CanSetVelocityConstants) || (this.gamepad1CanControlIntakeOrientation && gamepad1.b) ->
                ExtZoomBotConstants.ACTIVE_FLYWHEEL = false
            (watch_gamepad2_dpadUp() && !gamepad2CanSetVelocityConstants) || (this.gamepad1CanControlIntakeOrientation && watch_gamepad1_dpadUp()) ->
                ExtZoomBotConstants.ZOOM_VELOCITY += ExtZoomBotConstants.SMALL_CHANGE
            (watch_gamepad2_dpadDown() && !gamepad2CanSetVelocityConstants) || (this.gamepad1CanControlIntakeOrientation && watch_gamepad1_dpadDown()) ->
                ExtZoomBotConstants.ZOOM_VELOCITY -= ExtZoomBotConstants.SMALL_CHANGE
            (watch_gamepad2_dpadLeft() && !gamepad2.start && !gamepad2CanSetVelocityConstants) || (this.gamepad1CanControlIntakeOrientation && watch_gamepad1_dpadLeft()) ->
                ExtZoomBotConstants.ZOOM_VELOCITY -= ExtZoomBotConstants.LARGE_CHANGE
            (watch_gamepad2_dpadRight() && !gamepad2.start && !gamepad2CanSetVelocityConstants) || (this.gamepad1CanControlIntakeOrientation && watch_gamepad1_dpadRight()) ->
                ExtZoomBotConstants.ZOOM_VELOCITY += ExtZoomBotConstants.LARGE_CHANGE
        }

    when {
        (gamepad2.b && gamepad2CanSetVelocityConstants) -> ExtZoomBotConstants.ZOOM_VELOCITY = ExtZoomBotConstants.VELO_PRESET_1
        (gamepad2.a && gamepad2CanSetVelocityConstants) -> ExtZoomBotConstants.ZOOM_VELOCITY = ExtZoomBotConstants.VELO_PRESET_2
        (gamepad2.x && gamepad2CanSetVelocityConstants) -> ExtZoomBotConstants.ZOOM_VELOCITY = ExtZoomBotConstants.VELO_PRESET_3
    }

//        robot.zoomWheel.mode = ExtZoomBotConstants.ZOOM_MODE;

        if (ExtZoomBotConstants.ZOOM_MODE == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            robot.zoomWheel.power = when {
                ExtZoomBotConstants.ACTIVE_FLYWHEEL -> ExtZoomBotConstants.ZOOM_POWER
                gamepad2.left_trigger > 0.05 -> -gamepad2.left_trigger.toDouble()
                gamepad2.right_trigger > 0.05 -> gamepad2.right_trigger.toDouble()
                else -> 0.0
            }
        } else {
            robot.zoomWheel.velocity = (
                    if (ExtZoomBotConstants.ACTIVE_FLYWHEEL) ExtZoomBotConstants.ZOOM_VELOCITY else 0.0
                    )
        }

        robot.ringLoadServo.position = when {
            gamepad2.y || (this.gamepad1CanControlIntakeOrientation && gamepad1.y) -> ExtZoomBotConstants.RING_LOAD_SERVO_PUSH
            else -> ExtZoomBotConstants.RING_LOAD_SERVO_BACK
        }

        robot.ringTapper.move(when {
            gamepad2.x || (this.gamepad1CanControlIntakeOrientation && gamepad1.x) -> RingTapper.Position.TAP
            else -> RingTapper.Position.STORAGE
        })

        if (ExtZoomBotConstants.ENERGIZE) robot.zoomWheel.setMotorEnable() else robot.zoomWheel.setMotorDisable()
        robot.zoomWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ExtZoomBotConstants.VELOCITY_PID)
    }

    private fun controlMusic() {
        // if gamepad2 right stick button is toggled, toggle music
//        if (watch_gamepad2_rightStickButton.invoke()) {
//            if (musicPlayer.isPlaying()) musicPlayer.pause()
//            else musicPlayer.play()
//        }
        robot.blinkinController.update(
                if (ExtZoomBotConstants.ACTIVE_FLYWHEEL)
                when(ExtZoomBotConstants.ZOOM_VELOCITY) {
                    ExtZoomBotConstants.VELO_PRESET_1 -> ExtZoomBotConstants.VELO_PRESET_1_COLOR
                    ExtZoomBotConstants.VELO_PRESET_2 -> ExtZoomBotConstants.VELO_PRESET_2_COLOR
                    ExtZoomBotConstants.VELO_PRESET_3 -> ExtZoomBotConstants.VELO_PRESET_3_COLOR
                    0.0 -> RevBlinkinLedDriver.BlinkinPattern.BLACK
                    else -> RevBlinkinLedDriver.BlinkinPattern.GRAY
                }
                else RevBlinkinLedDriver.BlinkinPattern.BLACK
        )

//
    }

    private fun controlTelemetry() {
        telemetry.addData("TARGET flywheel velo", ExtZoomBotConstants.ZOOM_VELOCITY)
        telemetry.addData("CURRENT flywheel velo", robot.zoomWheel.velocity)
        telemetry.addData("PRESET of flywheel", when(ExtZoomBotConstants.ZOOM_VELOCITY) {
            ExtZoomBotConstants.VELO_PRESET_1 -> "POWER SHOT"
            ExtZoomBotConstants.VELO_PRESET_2 -> "HIGH GOAL"
            ExtZoomBotConstants.VELO_PRESET_3 -> "LONG RANGE"
            else -> "no preset"
        })
        telemetry.addLine()
        telemetry.addData("wobble grabber state", robot.wobbleGrabber.state)
        telemetry.addLine()
        telemetry.addData("controlled zoom op", ExtZoomBotConstants.ACTIVE_FLYWHEEL)
        telemetry.addData("controlled zoom speed", ExtZoomBotConstants.ZOOM_POWER)
        telemetry.addData("zoom motor mode", robot.zoomWheel.mode)
        telemetry.addData("zoom motor enabled", robot.zoomWheel.mode)
        telemetry.addLine()
        telemetry.addData("zoom motor power", robot.zoomWheel.power)
        telemetry.addData("zoom motor velo (rot)", (robot.zoomWheel.velocity / 28))
        telemetry.addData("zoom motor velo (rpm)", (robot.zoomWheel.velocity / 28) * 60)
        telemetry.addData("zoom motor velo (deg)", (robot.zoomWheel.velocity / 28) * 360)
        telemetry.addLine()
        telemetry.addData("original PID", originalPID)
        telemetry.addData("current PID", ExtZoomBotConstants.VELOCITY_PID)
        telemetry.addLine()
        telemetry.addData("gamepad 1 drivetrain control", canControlDrivetrainOrientation)
        telemetry.addData("gamepad 1 intake control", this.gamepad1CanControlIntakeOrientation)
        telemetry.addLine()
        telemetry.addData("robot heading (rad)", robot.imuControllerC.getHeading())
        telemetry.addData("robot heading (deg)", robot.imuControllerC.getHeading() * 180 / Math.PI)
        telemetry.update()
    }

    // functionality is explained throughout opmode; allows for encapsulation of button presses
    //   that toggle changes in variables
    // constructor parameter is a function (no parameters, returns Boolean)
}



