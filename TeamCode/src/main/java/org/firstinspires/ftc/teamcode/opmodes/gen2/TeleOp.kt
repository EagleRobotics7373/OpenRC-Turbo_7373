package org.firstinspires.ftc.teamcode.opmodes.gen2

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
import org.firstinspires.ftc.teamcode.library.vision.ultimategoal.PowerShotPipeline
import org.firstinspires.ftc.teamcode.library.vision.ultimategoal.UltimateGoalPowerShotConstants
import kotlin.math.absoluteValue

@TeleOp(name = "TeleOp Gen3", group = "Gen3 Basic")
open class TeleOp : OpMode() {
    // robot hardware declaration; assignment occures during init()
    lateinit var robot: ExtZoomBot

    // gamepad toggle button watchers, instantiated after opmode init
    private lateinit var watch_gamepad1_buttonY: ToggleButtonWatcher
    protected lateinit var watch_gamepad1_dpadDown: ToggleButtonWatcher
    protected lateinit var watch_gamepad1_dpadUp: ToggleButtonWatcher
    protected lateinit var watch_gamepad1_dpadLeft: ToggleButtonWatcher
    protected lateinit var watch_gamepad1_dpadRight: ToggleButtonWatcher
    private lateinit var watch_gamepad2_leftStickButton: ToggleButtonWatcher
    private lateinit var watch_gamepad2_rightStickButton: ToggleButtonWatcher
    private lateinit var watch_gamepad1_buttonA: ToggleButtonWatcher
    private lateinit var watch_gamepad1_buttonB: ToggleButtonWatcher
    private lateinit var watch_gamepad2_buttonA: ToggleButtonWatcher
    private lateinit var watch_gamepad2_buttonB: ToggleButtonWatcher
    private lateinit var watch_gamepad2_dpadLeft: ToggleButtonWatcher
    private lateinit var watch_gamepad2_dpadRight: ToggleButtonWatcher
    private lateinit var watch_gamepad2_dpadUp: ToggleButtonWatcher
    private lateinit var watch_gamepad2_dpadDown: ToggleButtonWatcher
    private lateinit var watch_gamepad1_leftBumper: ToggleButtonWatcher
    private lateinit var watch_gamepad1_rightBumper: ToggleButtonWatcher

    //    val musicPlayer = ExtDirMusicPlayer(ExtMusicFile.TETRIS)
    var playingMusic = false
    var elapsedTime: ElapsedTime? = null
    var last = 0.0
    var current = 0.0
    var zeroAngle = 0.0

    /*
    DRIVE CONSTANTS
     */

    // Current drive mode
    var driveMode: NewDriveMode = NewDriveMode.RobotCentric
        set(value) {
            if (driveMode == value) return
            lastDriveMode = driveMode; field=value
        }

    // Previous drive mode
    var lastDriveMode: NewDriveMode = driveMode

    // If current drive mode is reversed
    val reverse get() = ( driveMode is NewDriveMode.RobotCentric
            && (driveMode as? NewDriveMode.RobotCentric)?.reverse == true )

    // Current drivetrain speed
    var speed = 3

    /*
    OTHER VARIABLES
     */
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
    var useLEDs = true

    var container: OpenCvContainer<PowerShotPipeline>? = null

    override fun init() {
        // instantiate robot variables
        robot = ExtZoomBot(hardwareMap)
        robot.expansionhubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }
        // Instantiate toggle button watchers. Each statement below is calling a constructor with a single
        //   parameter, in this case being a function for calling the gamepad button. This does not set the
        //   current gamepad state only in the ToggleButtonWatcher; rather, it sets the ability for the
        //   ToggleButtonWatcher to invoke the gamepad values.
        watch_gamepad1_buttonY = ToggleButtonWatcher { gamepad1.y }
        watch_gamepad1_dpadDown = ToggleButtonWatcher { gamepad1.dpad_down }
        watch_gamepad1_dpadUp = ToggleButtonWatcher { gamepad1.dpad_up }
        watch_gamepad1_dpadLeft = ToggleButtonWatcher { gamepad1.dpad_left }
        watch_gamepad1_dpadRight = ToggleButtonWatcher { gamepad1.dpad_right }
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
//        container = VisionFactory.createOpenCv(VisionFactory.CameraType.WEBCAM_PLUS, hardwareMap, PowerShotPipeline())
        originalPID = robot.zoomWheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER)
    }

    override fun start() {
        container?.pipeline?.tracking = false
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
    }

    override fun stop() {
        // overrides OpMode.stop() to ensure hardware components stop
        super.stop()
        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK)
    }

    protected open fun controlDrivetrain() {
        if (driveMode is NewDriveMode.PowerShot) {

            NewDriveMode.PowerShot.target = when {
                gamepad1.dpad_left && canControlDrivetrainOrientation -> Position.LEFT
                gamepad1.dpad_up && canControlDrivetrainOrientation -> Position.CENTER
                gamepad1.dpad_right && canControlDrivetrainOrientation -> Position.RIGHT
                else -> Position.NULL
            }

            when (NewDriveMode.PowerShot.target) {
                Position.NULL -> {

                    robot.holonomic.setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

                    val input = ExtZoomBotConstants.PS_ROTATE_SPEED * gamepad1.right_stick_x
                    robot.frontLeftMotor.power = if (ExtZoomBotConstants.PS_FRONT_LEFT) input else 0.0
                    robot.frontRightMotor.power = if (ExtZoomBotConstants.PS_FRONT_RIGHT) input else 0.0
                    robot.backLeftMotor.power = if (ExtZoomBotConstants.PS_BACK_LEFT) input else 0.0
                    robot.backRightMotor.power = if (ExtZoomBotConstants.PS_BACK_RIGHT) input else 0.0

                }
                else -> {

                    val current: Double = (container?.pipeline?.distances?.get(NewDriveMode.PowerShot.targetAsInt)?.distance) ?: 0.0
                    val target = 0.0
                    val error = target - current
                    val output = error * -ExtZoomBotConstants.PS_SEEK.p
                    val actualOutput =
                            if (output < 0) output.coerceIn(-ExtZoomBotConstants.PS_SEEK_MAX, -ExtZoomBotConstants.PS_SEEK_MIN)
                            else if (output > 0) output.coerceIn(ExtZoomBotConstants.PS_SEEK_MIN, ExtZoomBotConstants.PS_SEEK_MAX)
                            else 0.0

                    robot.frontLeftMotor.power = if (ExtZoomBotConstants.PS_FRONT_LEFT) actualOutput else 0.0
                    robot.frontRightMotor.power = if (ExtZoomBotConstants.PS_FRONT_RIGHT) actualOutput else 0.0
                    robot.backLeftMotor.power = if (ExtZoomBotConstants.PS_BACK_LEFT) actualOutput else 0.0
                    robot.backRightMotor.power = if (ExtZoomBotConstants.PS_BACK_RIGHT) actualOutput else 0.0
                }

            }

            if (gamepad1.left_stick_x.absoluteValue > 0.15 || gamepad1.left_stick_y.absoluteValue > 0.15) driveMode = lastDriveMode

            if (canControlDrivetrainOrientation && gamepad1.b) UltimateGoalPowerShotConstants.ALLIANCE_COLOR = AllianceColor.RED
            else if (canControlDrivetrainOrientation && gamepad1.x) UltimateGoalPowerShotConstants.ALLIANCE_COLOR = AllianceColor.BLUE

        } else {
            // Set x, y, and z inputs:
            //    Use Double.rangeBuffer to set motor power dead-band
            //    Use Double.times() for variable speed. (This functional method works the same as doing x * y in Java)
            //    Use Double.reverseIf() on x and y to easily reverse input direction
            val x = gamepad1.left_stick_x.toDouble().rangeBuffer(-0.1, 0.1, 0.0).times(0.33 * speed).reverseIf(reverse)
            val y = -gamepad1.left_stick_y.toDouble().rangeBuffer(-0.1, 0.1, 0.0).times(0.33 * speed).reverseIf(reverse)
            val z = gamepad1.right_stick_x.toDouble().rangeBuffer(-0.1, 0.1, 0.0).times(0.33 * speed)

            // set gamepad inputs to robot output
            robot.holonomic.runWithoutEncoderVectored(x, y, z, if (driveMode == NewDriveMode.FieldCentric) zeroAngle - robot.imuControllerC.getHeading() else 0.0)

            // read 'a' and 'b' buttons to reverse robot orientation
            if (canControlDrivetrainOrientation && gamepad1.a)
                driveMode = NewDriveMode.RobotCentric.apply { reverse = false }
            else if (canControlDrivetrainOrientation && gamepad1.b)
                driveMode = NewDriveMode.RobotCentric.apply { reverse = true }
            else if (canControlDrivetrainOrientation && gamepad1.x)
                driveMode = NewDriveMode.FieldCentric

            // reverse robot orientation using toggle feature in addition to static buttons
            // if y button is pressed, make reverse variable opposite of what it is now
            //   then wait until y is released before letting it change the variable again
            if (canControlDrivetrainOrientation && watch_gamepad1_buttonY.invoke())
                NewDriveMode.RobotCentric.reverse = !NewDriveMode.RobotCentric.reverse

            // Change to Power Shot mode if the right stick button is pressed
            if (gamepad1.right_stick_button) driveMode = NewDriveMode.PowerShot
        }


        // reset IMU baseline if left stick button is pressed
        if (gamepad1.left_stick_button) NewDriveMode.FieldCentric.zeroAngle = robot.imuControllerC.getHeading()



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

        when {
            (watch_gamepad2_leftStickButton()) || (watch_gamepad1_leftBumper() && gamepad1.start) -> robot.wobbleGrabber.prevState()
            (watch_gamepad2_rightStickButton()) || (watch_gamepad1_rightBumper() && gamepad1.start) -> robot.wobbleGrabber.nextState()
        }
        robot.wobbleGrabber.update()

    }

    private fun controlIntakeMechanism() {
        val power = when {
            gamepad2.left_trigger > 0.05 -> -gamepad2.left_trigger.toDouble()
            gamepad2.right_trigger > 0.05 -> gamepad2.right_trigger.toDouble()
            gamepad1.left_trigger > 0.05 -> -gamepad1.left_trigger.toDouble()
            gamepad1.right_trigger > 0.05 -> gamepad1.right_trigger.toDouble()
            else -> 0.0
        }
        robot.intakeStage1.power = if (gamepad2.left_stick_y.absoluteValue > 0.05) gamepad2.left_stick_y.toDouble() else power
        robot.intakeStage2.power = (if (gamepad2.right_stick_y.absoluteValue > 0.05) gamepad2.right_stick_y.toDouble() else power) * ExtZoomBotConstants.STAGE2_POWER_LIMIT

        when {
            (gamepad2.a && !gamepad2CanSetVelocityConstants) || (this.gamepad1CanControlIntakeOrientation && gamepad1.a) ->
                ExtZoomBotConstants.ACTIVE_FLYWHEEL = true
            (gamepad2.b && !gamepad2CanSetVelocityConstants) || (this.gamepad1CanControlIntakeOrientation && gamepad1.b) ->
                ExtZoomBotConstants.ACTIVE_FLYWHEEL = false
            (watch_gamepad2_dpadUp() && !gamepad2CanSetVelocityConstants) || (this.gamepad1CanControlIntakeOrientation && watch_gamepad1_dpadUp()) ->
                ExtZoomBotConstants.ZOOM_VELOCITY += ExtZoomBotConstants.SMALL_CHANGE
            (watch_gamepad2_dpadDown() && !gamepad2CanSetVelocityConstants) || (this.gamepad1CanControlIntakeOrientation && watch_gamepad1_dpadDown()) ->
                ExtZoomBotConstants.ZOOM_VELOCITY -= ExtZoomBotConstants.SMALL_CHANGE
            (watch_gamepad2_dpadLeft() && !gamepad2.start && !gamepad2CanSetVelocityConstants)
                    || (this.gamepad1CanControlIntakeOrientation && driveMode !is NewDriveMode.PowerShot && watch_gamepad1_dpadLeft()) ->
                ExtZoomBotConstants.ZOOM_VELOCITY -= ExtZoomBotConstants.LARGE_CHANGE
            (watch_gamepad2_dpadRight() && !gamepad2.start && !gamepad2CanSetVelocityConstants)
                    || (this.gamepad1CanControlIntakeOrientation && driveMode !is NewDriveMode.PowerShot && watch_gamepad1_dpadRight()) ->
                ExtZoomBotConstants.ZOOM_VELOCITY += ExtZoomBotConstants.LARGE_CHANGE
        }

        when {
            (gamepad2.b && gamepad2CanSetVelocityConstants) -> ExtZoomBotConstants.ZOOM_VELOCITY = ExtZoomBotConstants.VELO_PRESET_1
            (gamepad2.a && gamepad2CanSetVelocityConstants) -> ExtZoomBotConstants.ZOOM_VELOCITY = ExtZoomBotConstants.VELO_PRESET_2
            (gamepad2.x && gamepad2CanSetVelocityConstants) -> ExtZoomBotConstants.ZOOM_VELOCITY = ExtZoomBotConstants.VELO_PRESET_3
        }

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

        robot.ringTapThru.move(when {
            gamepad2.x || (this.gamepad1CanControlIntakeOrientation && gamepad1.x) -> RingTapper.Position.TAP
            else -> RingTapper.Position.STORAGE
        })

        if (ExtZoomBotConstants.ENERGIZE) robot.zoomWheel.setMotorEnable() else robot.zoomWheel.setMotorDisable()
        robot.zoomWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ExtZoomBotConstants.VELOCITY_PID)

        robot.deflectionServo.position = ExtZoomBotConstants.SERVO_DEFLECTION_POS
    }

    private fun controlMusic() {
        robot.blinkinController.update(
                if (ExtZoomBotConstants.ACTIVE_FLYWHEEL)
                    when (ExtZoomBotConstants.ZOOM_VELOCITY) {
                        ExtZoomBotConstants.VELO_PRESET_1 -> ExtZoomBotConstants.VELO_PRESET_1_COLOR
                        ExtZoomBotConstants.VELO_PRESET_2 -> ExtZoomBotConstants.VELO_PRESET_2_COLOR
                        ExtZoomBotConstants.VELO_PRESET_3 -> ExtZoomBotConstants.VELO_PRESET_3_COLOR
                        0.0 -> RevBlinkinLedDriver.BlinkinPattern.BLACK
                        else -> RevBlinkinLedDriver.BlinkinPattern.GRAY
                    }
                else RevBlinkinLedDriver.BlinkinPattern.BLACK
        )
    }

    var lastR = System.currentTimeMillis()
    var lastE = 0.0
    var lastRe = 0.0
    private fun controlTelemetry() {
//        val now = System.currentTimeMillis()
//        val nearbyFlywheelVelo = ((robot.zoomWheel.currentPosition - lastE) * 1000 / (now - lastR)).toInt()
//        val reportedFlywheelVelo = robot.zoomWheel.velocity.toInt()
//        val velocitiesAreSameSign = (nearbyFlywheelVelo xor reportedFlywheelVelo) > 0
//        val adjustmentFactor = (round(((nearbyFlywheelVelo / 32767) - (0.5 * (if (nearbyFlywheelVelo >= 0) 1 else -1)) / 2)) * 2 + (if (velocitiesAreSameSign) 0 else 1))
//        val adjustedFlywheelVelo = 32767 * adjustmentFactor + if (velocitiesAreSameSign) reportedFlywheelVelo else (-32767 - reportedFlywheelVelo * -1)

        telemetry.addData("Drive Mode", driveMode)
        container?.pipeline?.tracking = driveMode == NewDriveMode.PowerShot
        if (driveMode is NewDriveMode.PowerShot) {
            val distances = container?.pipeline?.distances
            telemetry.addData("Power Shot Status", "Active")
            telemetry.addData("Alliance Color", UltimateGoalPowerShotConstants.ALLIANCE_COLOR)
            telemetry.addData("Distances Available", distances != null)
            if (distances != null) {
                telemetry.addData("LEFT", distances[0])
                telemetry.addData("CENTER", distances[1])
                telemetry.addData("RIGHT", distances[2])
            }

        }
        telemetry.addLine()

        telemetry.addData("TARGET flywheel velo", ExtZoomBotConstants.ZOOM_VELOCITY)
        telemetry.addData("CURRENT flywheel velo", robot.zoomWheel.velocity)
//        telemetry.addData("ADJUSTED flywheel velo", adjustedFlywheelVelo)
//        telemetry.addData("ADJUSTED flywheel velo(rot)", adjustedFlywheelVelo / 1440)
//        telemetry.addData("NEARBY flywheel velo", nearbyFlywheelVelo)
        telemetry.addData("PRESET of flywheel", when (ExtZoomBotConstants.ZOOM_VELOCITY) {
            ExtZoomBotConstants.VELO_PRESET_1 -> "POWER SHOT"
            ExtZoomBotConstants.VELO_PRESET_2 -> "HIGH GOAL"
            ExtZoomBotConstants.VELO_PRESET_3 -> "LONG RANGE"
            else -> "no preset"
        })
//        telemetry.addData("ADJUSTMENT factor", adjustmentFactor)
        telemetry.addLine()
        telemetry.addData("wobble grabber state", robot.wobbleGrabber.state)
        telemetry.addLine()
        telemetry.addData("controlled zoom op", ExtZoomBotConstants.ACTIVE_FLYWHEEL)
        telemetry.addData("controlled zoom speed", ExtZoomBotConstants.ZOOM_POWER)
        telemetry.addData("zoom motor mode", robot.zoomWheel.mode)
        telemetry.addData("zoom motor enabled", robot.zoomWheel.mode)
        telemetry.addLine()
        telemetry.addData("zoom motor power", robot.zoomWheel.power)
        telemetry.addData("zoom motor velo (rot)", (robot.zoomWheel.velocity / 1440))
        telemetry.addData("zoom motor velo (rpm)", (robot.zoomWheel.velocity / 1440) * 60)
        telemetry.addLine()
        telemetry.addData("original PID", originalPID)
        telemetry.addData("current PID", ExtZoomBotConstants.VELOCITY_PID)
        telemetry.addLine()
        telemetry.addData("gamepad 1 drivetrain control", canControlDrivetrainOrientation)
        telemetry.addData("gamepad 1 intake control", this.gamepad1CanControlIntakeOrientation)
        telemetry.addLine()
        telemetry.addData("robot heading (rad)", robot.imuControllerC.getHeading())
        telemetry.addData("robot heading (deg)", robot.imuControllerC.getHeading() * 180 / Math.PI)
//        lastR = now
//        lastE = robot.zoomWheel.currentPosition.toDouble()
    }

    // functionality is explained throughout opmode; allows for encapsulation of button presses
    //   that toggle changes in variables
    // constructor parameter is a function (no parameters, returns Boolean)



    sealed class NewDriveMode {
        object RobotCentric : NewDriveMode() {
            var reverse = false

            override fun toString(): String = "RobotCentric: ${if (reverse) "REVERSE" else "FORWARD"}"
        }
        object FieldCentric : NewDriveMode() {
            var zeroAngle = 0.0

            override fun toString(): String = "FieldCentric: zeroAngle=$zeroAngle"
        }
        object PowerShot : NewDriveMode() {

            var target: Position = Position.CENTER

            val targetAsInt: Int
            get() = when(target) {
                Position.LEFT -> 0
                Position.CENTER -> 1
                Position.RIGHT -> 2
                Position.NULL -> -1
            }

            override fun toString(): String = ("PowerShot"
                    + if (target != Position.NULL) " to $target" else "")
        }
    }
}



