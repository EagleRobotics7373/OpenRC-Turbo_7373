package org.firstinspires.ftc.teamcode.opmodes.gen2

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin.MenuItemEnumDelegate
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtRingPlaceBot
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.WobbleGrabber

@TeleOp(name="TeleOp RD Gen2", group="Gen2 Basic")
open class TeleOpRD : OpMode() {
    // robot hardware declaration; assignment occures during init()
    lateinit var robot : ExtRingPlaceBot

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
        robot = ExtRingPlaceBot(hardwareMap)
        robot.expansionhubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.AUTO}
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


    }

    override fun loop() {
        // call methods for each individual system for streamlined navigation
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
        robot.holonomicRR.runWithoutEncoder(x, y, z)

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

    /**
     * Control wobble grabber
     */
    private fun controlOtherDevices() {
//        when {
//            gamepad2.y -> robot.wobbleGrabber.releaseGrab()     // y is for yeet
//            gamepad2.x -> robot.wobbleGrabber.grab()
//            gamepad2.b -> robot.wobbleGrabber.midGrab()
//        }

        if (gamepad2.left_bumper) {
            when {
                gamepad2.a -> robot.wobbleGrabber.pivot(WobbleGrabber.PivotPosition.GRAB)
                gamepad2.b -> robot.wobbleGrabber.pivot(WobbleGrabber.PivotPosition.STORAGE)
                gamepad2.x -> robot.wobbleGrabber.pivot(WobbleGrabber.PivotPosition.PERPENDICULAR)
                gamepad2.y -> robot.wobbleGrabber.pivot(WobbleGrabber.PivotPosition.YEET)
            }
        } else if (gamepad2.right_bumper) {
            when {
                gamepad2.a -> robot.wobbleGrabber.grab(WobbleGrabber.GrabPosition.GRAB)
                gamepad2.b -> robot.wobbleGrabber.grab(WobbleGrabber.GrabPosition.STORAGE)
                gamepad2.x -> robot.wobbleGrabber.grab(WobbleGrabber.GrabPosition.MID_GRAB)
            }
        }



    }

    // set PID control coefficients for auto-intake raise in controlIntakeMechanism()
    // this should eventually be moved
    private fun controlIntakeMechanism() {
        robot.intakeSystem.manualMoveIntake(gamepad2.left_stick_y.toDouble())

        robot.intakeSystem.manualRingMotor(
                when {
                    gamepad2.left_trigger  > 0.05 -> -gamepad2.left_trigger.toDouble()
                    gamepad2.right_trigger > 0.05 -> gamepad2.right_trigger.toDouble()
                    else                          -> 0.0
                }
        )
    }

    private fun controlMusic() {
        // if gamepad2 right stick button is toggled, toggle music
//        if (watch_gamepad2_rightStickButton.call()) {
//            if (musicPlayer.isPlaying()) musicPlayer.pause()
//            else musicPlayer.play()
//        }
    }

    private fun controlTelemetry() {

    }

    // functionality is explained throughout opmode; allows for encapsulation of button presses
    //   that toggle changes in variables
    // constructor parameter is a function (no parameters, returns Boolean)
}



