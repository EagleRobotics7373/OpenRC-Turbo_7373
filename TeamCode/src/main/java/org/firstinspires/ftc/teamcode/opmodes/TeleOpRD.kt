package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot
import kotlin.math.absoluteValue

@TeleOp(name="TeleOp RD", group="basic")
open class TeleOpRD : OpMode() {
    // robot hardware declaration; assignment occures during init()
    lateinit var robot : BasicRobot

    // gamepad toggle button watchers, instantiated after opmode init
    protected lateinit var watch_gamepad1_buttonY : ToggleButtonWatcher
    protected lateinit var watch_gamepad1_dpadDown : ToggleButtonWatcher
    protected lateinit var watch_gamepad1_dpadUp : ToggleButtonWatcher
    protected lateinit var watch_gamepad2_rightStickButton : ToggleButtonWatcher

    val musicPlayer = ExtDirMusicPlayer(ExtMusicFile.UNITY)
    var playingMusic = false
    var elapsedTime : ElapsedTime? = null
    var last = 0.0
    var current = 0.0
    var reverse = false
    var speed = 3

    override fun init() {
        // instantiate robot variables
        robot = BasicRobot(hardwareMap)

        // Instantiate toggle button watchers. Each statement below is calling a constructor with a single
        //   parameter, in this case being a function for calling the gamepad button. This does not set the
        //   current gamepad state only in the ToggleButtonWatcher; rather, it sets the ability for the
        //   ToggleButtonWatcher to call the gamepad values.
        watch_gamepad1_buttonY = ToggleButtonWatcher {gamepad1.y}
        watch_gamepad1_dpadDown = ToggleButtonWatcher {gamepad1.dpad_down}
        watch_gamepad1_dpadUp = ToggleButtonWatcher {gamepad1.dpad_up}
        watch_gamepad2_rightStickButton = ToggleButtonWatcher { gamepad2.right_stick_button }
    }

    override fun loop() {
        // call methods for each individual system for streamlined navigation
        controlDrivetrain()
        controlFoundationGrabbers()
        controlIntakeMechanism()
        controlTelemetry()
        controlMusic()
    }

    override fun stop() {
        // overrides OpMode.stop() to ensure hardware components and music player stop
        super.stop()
        musicPlayer.stop()
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
        robot.holonomic.runWithoutEncoder(x, y, z)

        // read 'a' and 'b' buttons to reverse robot orientation
        if (gamepad1.a) reverse = false
        else if (gamepad1.b) reverse = true

        // reverse robot orientation using toggle feature in addition to static buttons
        // if y button is pressed, make reverse variable opposite of what it is now
        //   then wait until y is released before letting it change the variable again
        if (watch_gamepad1_buttonY.call()) reverse = !reverse

        // increase or decrease robot speed variable using toggle system
        // speed variable should be between 1 and 3
        //    3 = max power (0.33 * 3 * input)
        //    2 = mid power (0.33 * 2 * input)
        //    1 = low power (0.33 * 1 * input)
        if (watch_gamepad1_dpadDown.call() and (speed > 1)) speed--
        if (watch_gamepad1_dpadUp.call() and (speed < 3)) speed++


    }

    private fun controlFoundationGrabbers() {
        // raise or lower foundation grabbers using gamepad2 dpad
        if (gamepad2.dpad_down) robot.foundationGrabbers.lock()
        else if (gamepad2.dpad_up) robot.foundationGrabbers.unlock()
    }

    // set PID control coefficients for auto-intake raise in controlIntakeMechanism()
    // this should eventually be moved
    private var intakeP = 4.0
    private var intakeI = 0.000
    private var intakeD = 0.20
    private val voltageTarget = 1.257
    private var lastRuntime = -1.0
    private var d = 0.0
    private var lastError = 0.0
    private var errorSum = 0.0

    private fun controlIntakeMechanism() {
        // control intake arm up/down
        // This variable is being set by a Kotlin expression -
        //   meaning the variable will be set to last value in an if/else/when branch
        val input =
                // if left stick button is pressed, set input to auto-raise value
                if (gamepad2.left_stick_button) {

                    //use pid loop
                    val error = voltageTarget-robot.intakePivotPotentiometer.voltage
                    if (lastRuntime==-1.0) lastRuntime = runtime
                    errorSum += error*runtime-lastRuntime

                    d = error-lastError
                    lastRuntime = runtime
                    lastError = error

                    // this is the last expression in this if branch, so this would be assigned to 'input':
                    -(intakeP * error + intakeI * errorSum + intakeD * d)
                }
                else  {
                    // this is the last statement in this else branch, so this would be assigned to 'input':
                    gamepad2.left_stick_y.toDouble()
                }

        // set intake pivot motor power using Kotlin expression assignment
        robot.intakePivotMotor.power =
                when {
                    // if pressing right gamepad bumper, use constant power of 0.02
                    (gamepad2.right_bumper)-> if (input > 0.0) input+0.02 else 0.02 - -input*0.04

                    // if input is greater than 0, give full increase starting at constant power of 0.12
                    input > 0.0 -> input+0.12

                    // otherwise, give constant power at 0.12 and diminished decrease (0.12 + input * 0.14)
                    else -> 0.12 + input*0.14
                }/*.rangeBuffer(-0.10, 0.10, 0.0)*/

        // if gamepad2 right stick y is being used outside of dead-band, give it priority control over intake wheels
        if (gamepad2.right_stick_y.toDouble().absoluteValue > 0.10) robot.intakeBlockManipulator.power = -gamepad2.right_stick_y.toDouble()
        // if gamepad1 right trigger is pressed, let it drive wheels to intake blocks
        else if (gamepad1.right_trigger > 0.03) robot.intakeBlockManipulator.power = gamepad1.right_trigger.toDouble()
        // if gamepad1 left trigger is pressed, let it drive wheels to release blocks
        else if (gamepad1.left_trigger > 0.03) robot.intakeBlockManipulator.power = -gamepad1.left_trigger.toDouble()
        // otherwise do not drive intake wheels
        else robot.intakeBlockManipulator.power = 0.0

        // control grabbing servo using basic gamepad input
        if (gamepad2.x or gamepad1.right_bumper) robot.intakeBlockGrabber.hold()
        else if (gamepad2.y or gamepad1.left_bumper) robot.intakeBlockGrabber.release()
        else if (gamepad2.a or gamepad1.right_stick_button) robot.intakeBlockGrabber.mid()

    }

    private fun controlMusic() {
        // if gamepad2 right stick button is toggled, toggle music
        if (watch_gamepad2_rightStickButton.call()) {
            if (musicPlayer.isPlaying()) musicPlayer.pause()
            else musicPlayer.play()
        }
    }

    private fun controlTelemetry() {
        telemetry.addData("Intake manipulator power", robot.intakeBlockManipulator.power)
        telemetry.addData("Intake pivot power", robot.intakePivotMotor.power)
//        telemetry.addData("Potentiometer voltage", robot.intakePivotPotentiometer.voltage)
//        telemetry.addData("Potentiometer max v", robot.intakePivotPotentiometer.maxVoltage)
//        telemetry.addData("Front dist gD()", robot.frontDistanceSensor.getDistance(DistanceUnit.CM))
//        telemetry.addData("Front dist cU()", robot.frontDistanceSensor.cmUltrasonic())
        telemetry.addData("Speed", speed)
        telemetry.addData("Reverse", reverse)
//        telemetry.addData("Hue in range?", robot.intakeBlockCSensor.rhue in 0.32..0.41)
//        telemetry.addData("hue", robot.intakeBlockCSensor.rhue)
//        telemetry.addData("DS distance", robot.intakeBlockDSensor.getDistance(DistanceUnit.MM))
        telemetry.update()
    }

    // this function is not used
    fun setCoeffs(p: Double, i: Double, d: Double) {
        intakeP = p
        intakeI = i
        intakeD = d
    }

    // functionality is explained throughout opmode; allows for encapsulation of button presses
    //   that toggle changes in variables
    // constructor parameter is a function (no parameters, returns Boolean)
    protected class ToggleButtonWatcher(private val getMethod: ()->Boolean) {
        private var lastState : Boolean = getMethod()
        fun call(): Boolean {
            if (getMethod()) {
                if (!lastState) {
                    lastState = true
                    return true
                }
            } else lastState = false
            return false
        }
    }
}



