package org.firstinspires.ftc.teamcode.opmodes.gen2

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtMisumiRobot
import kotlin.math.absoluteValue

@TeleOp(name="TeleOp RD Gen2", group="Gen2 Basic")
open class TeleOpRD : OpMode() {
    // robot hardware declaration; assignment occures during init()
    lateinit var robot : ExtMisumiRobot

    // gamepad toggle button watchers, instantiated after opmode init
    protected lateinit var watch_gamepad1_buttonY : ToggleButtonWatcher
    protected lateinit var watch_gamepad1_dpadDown : ToggleButtonWatcher
    protected lateinit var watch_gamepad1_dpadUp : ToggleButtonWatcher
    protected lateinit var watch_gamepad2_rightStickButton : ToggleButtonWatcher
    protected lateinit var watch_gamepad2_buttonB : ToggleButtonWatcher

    val musicPlayer = ExtDirMusicPlayer(ExtMusicFile.TETRIS)
    var playingMusic = false
    var elapsedTime : ElapsedTime? = null
    var last = 0.0
    var current = 0.0
    var reverse = false
    var speed = 3

    var intakeLiftBaseline = 0
    var intakePivotBaseline = 0
    var reverseIntakeLift = true
    var useLEDs             = true


    override fun init() {
        // instantiate robot variables
        robot = ExtMisumiRobot(hardwareMap)
        robot.expansionhubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.OFF}
        // Instantiate toggle button watchers. Each statement below is calling a constructor with a single
        //   parameter, in this case being a function for calling the gamepad button. This does not set the
        //   current gamepad state only in the ToggleButtonWatcher; rather, it sets the ability for the
        //   ToggleButtonWatcher to call the gamepad values.
        watch_gamepad1_buttonY = ToggleButtonWatcher {gamepad1.y}
        watch_gamepad1_dpadDown = ToggleButtonWatcher {gamepad1.dpad_down}
        watch_gamepad1_dpadUp = ToggleButtonWatcher {gamepad1.dpad_up}
        watch_gamepad2_rightStickButton = ToggleButtonWatcher { gamepad2.right_stick_button }
        watch_gamepad2_buttonB = ToggleButtonWatcher { !gamepad2.right_bumper && gamepad2.b }

        intakeLiftBaseline = 0
        intakePivotBaseline = robot.intakePivot.currentPosition


    }

    override fun init_loop() {
        robot.autoBlockIntakeFront.pivotUp()
        robot.autoBlockIntakeRear.pivotUp()
    }

    override fun loop() {
        // call methods for each individual system for streamlined navigation
        controlDrivetrain()
        controlFoundationGrabbers()
        controlOtherDevices()
        controlIntakeMechanism()
        controlTelemetry()
        controlMusic()
//        robot.holonomicRR.update()
        if (!useLEDs) robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK)
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
        when {
            gamepad2.dpad_up -> robot.foundationGrabbersFront.unlock()
            gamepad2.dpad_down -> robot.foundationGrabbersFront.lock()
            gamepad2.dpad_left -> useLEDs = false
            gamepad2.dpad_right -> useLEDs = true
        }
    }

    private fun controlOtherDevices() {
        if (gamepad2.right_bumper) {
            when {
                gamepad2.b -> {

                    robot.autoBlockIntakeFront.pivotUp()
                    robot.autoBlockIntakeFront.grabBlock()
                    robot.autoBlockIntakeRear.pivotUp()
                    robot.autoBlockIntakeRear.grabBlock()
                }
//                gamepad2.x -> robot.capstonePlacer.moveInside()
//                gamepad2.y -> robot.capstonePlacer.moveDeploy()
//                gamepad2.a -> robot.capstonePlacer.moveIn18()

            }
        }
//        if (gamepad2.left_bumper) {
//            robot.autoBlockIntakeFront.pivotDown()
//            robot.autoBlockIntakeFront.releaseBlock()
//            robot.autoBlockIntakeRear.pivotDown()
//            robot.autoBlockIntakeRear.releaseBlock()
//        }
    }

    // set PID control coefficients for auto-intake raise in controlIntakeMechanism()
    // this should eventually be moved
    private fun controlIntakeMechanism() {
        // control intake arm up/down
        // This variable is being set by a Kotlin expression -
        //   meaning the variable will be set to last value in an if/else/when branch
        var inputSpoolPower =
                if (gamepad2.left_stick_y.absoluteValue > 0.05)
                    -gamepad2.left_stick_y.toDouble()
                else if (gamepad1.left_trigger > 0.05)
                    -gamepad1.left_trigger.toDouble()
                else if (gamepad1.right_trigger > 0.05)
                    gamepad1.right_trigger.toDouble()
                else
                    0.0

        val intakeLiftCurrent = robot.intakeLiftRight.currentPosition

        if (reverseIntakeLift) inputSpoolPower *= -1.0

        if (intakeLiftCurrent < -800) inputSpoolPower += 0.13

        if (watch_gamepad2_buttonB.call()) reverseIntakeLift = !reverseIntakeLift

        var ledFlashing = true

        val inputSpoolPowerMod =
                if (inputSpoolPower < 0.0 && !gamepad2.left_bumper)
                    (if (intakeLiftCurrent > intakeLiftBaseline) {
                        0.0
                    }
                    else if (intakeLiftCurrent + 75 > intakeLiftBaseline) 0.15
                    else if (intakeLiftCurrent + 500 > intakeLiftBaseline) 0.25
                    else if (intakeLiftCurrent + 800 > intakeLiftBaseline) 0.30
                    else if (intakeLiftCurrent + 1200 > intakeLiftBaseline) 0.35
                    else if (intakeLiftCurrent + 2000 > intakeLiftBaseline) 0.45
                    else 1.0)
                else 1.0


        if (useLEDs) {
            if (intakeLiftCurrent + 45 < intakeLiftBaseline) robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED)
            else robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE)
        }


        inputSpoolPower *= inputSpoolPowerMod

        (robot.intakeLiftLeft as DcMotor).power = inputSpoolPower
        (robot.intakeLiftRight as DcMotor).power = -inputSpoolPower

        // control grabbing servo using basic gamepad input
        if ((gamepad2.x && !gamepad2.right_bumper) || gamepad1.right_bumper) robot.intakeBlockGrabber.hold()
        else if ((gamepad2.y && !gamepad2.right_bumper) || gamepad1.left_bumper) robot.intakeBlockGrabber.release()
        else if ((gamepad2.a && !gamepad2.right_bumper) || gamepad1.right_stick_button) robot.intakeBlockGrabber.mid()

        if (gamepad2.left_stick_button) { //reset the intake baselines
            intakeLiftBaseline = robot.intakeLiftRight.currentPosition
            intakePivotBaseline = robot.intakePivot.currentPosition
        }

        val inputIntakePivotPower = gamepad2.right_stick_y.toDouble().rangeBuffer(-0.2, 0.2, 0.0)
        val intakePivotCurrent = robot.intakePivot.currentPosition
        val modifiedPosition = intakePivotCurrent - intakePivotBaseline

        telemetry.addData("intake pivot current", intakePivotCurrent)
        telemetry.addData("intake lift baseline", intakeLiftBaseline)
        telemetry.addData("modified pivot pos", modifiedPosition)

        if (inputIntakePivotPower != 0.0 || robot.intakePivot.mode == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            if (robot.intakePivot.mode == DcMotor.RunMode.RUN_TO_POSITION) robot.intakePivot.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            robot.intakePivot.power = inputIntakePivotPower
        }
        if (watch_gamepad2_rightStickButton.call()) {
            robot.intakePivot.targetPosition =
                    if (modifiedPosition < 500) intakePivotBaseline + 1000
                    else intakePivotBaseline

            robot.intakePivot.power = 0.7

            robot.intakePivot.mode = DcMotor.RunMode.RUN_TO_POSITION

        }

    }

    private fun controlMusic() {
        // if gamepad2 right stick button is toggled, toggle music
//        if (watch_gamepad2_rightStickButton.call()) {
//            if (musicPlayer.isPlaying()) musicPlayer.pause()
//            else musicPlayer.play()
//        }
    }

    private fun controlTelemetry() {
        telemetry.addData("misumi power", robot.intakeLiftRight.power)
        telemetry.addData("misumi pos", robot.intakeLiftRight.currentPosition)
    }

    // functionality is explained throughout opmode; allows for encapsulation of button presses
    //   that toggle changes in variables
    // constructor parameter is a function (no parameters, returns Boolean)
}



