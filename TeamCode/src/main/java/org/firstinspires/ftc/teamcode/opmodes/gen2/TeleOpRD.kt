package org.firstinspires.ftc.teamcode.opmodes.gen2

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.ExtRingPlaceBot
import org.firstinspires.ftc.teamcode.library.robot.systems.intakegen2.FullIntakeSystem
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.WobbleGrabber
import org.firstinspires.ftc.teamcode.library.vision.base.OpenCvContainer
import org.firstinspires.ftc.teamcode.library.vision.ultimategoal.IntakeRingViewingPipeline
import kotlin.math.absoluteValue

@TeleOp(name="TeleOp RD Gen2", group="Gen2 Basic")
open class TeleOpRD : OpMode() {
    // robot hardware declaration; assignment occures during init()
    lateinit var robot : ExtRingPlaceBot

    // gamepad toggle button watchers, instantiated after opmode init
    private lateinit var watch_gamepad1_buttonY : ToggleButtonWatcher
    protected lateinit var watch_gamepad1_dpadDown : ToggleButtonWatcher
    protected lateinit var watch_gamepad1_dpadUp : ToggleButtonWatcher
    private lateinit var watch_gamepad2_leftStickButton : ToggleButtonWatcher
    private lateinit var watch_gamepad2_rightStickButton : ToggleButtonWatcher
    private lateinit var watch_gamepad1_buttonA : ToggleButtonWatcher
    private lateinit var watch_gamepad1_buttonB : ToggleButtonWatcher
    private lateinit var watch_gamepad2_buttonA : ToggleButtonWatcher
    private lateinit var watch_gamepad2_buttonB : ToggleButtonWatcher
    private lateinit var watch_gamepad2_dpadLeft : ToggleButtonWatcher
    private lateinit var watch_gamepad2_dpadRight : ToggleButtonWatcher
    private lateinit var watch_gamepad1_leftBumper : ToggleButtonWatcher
    private lateinit var watch_gamepad1_rightBumper : ToggleButtonWatcher

    val musicPlayer = ExtDirMusicPlayer(ExtMusicFile.TETRIS)
    var playingMusic = false
    var elapsedTime : ElapsedTime? = null
    var last = 0.0
    var current = 0.0
    var reverse = false
    var speed = 3
    var lastCycleTime = System.currentTimeMillis()

    protected val canControlIntakeOrientation: Boolean
    get() = (gamepad1.left_bumper && gamepad1.right_bumper && !gamepad1.start)

    var intakeLiftBaseline = 0
    var intakePivotBaseline = 0
    var reverseIntakeLift = true
    var useLEDs             = true

    var container : OpenCvContainer<IntakeRingViewingPipeline>? = null

    override fun init() {
        // instantiate robot variables
        robot = ExtRingPlaceBot(hardwareMap)
        robot.expansionhubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL}
        // Instantiate toggle button watchers. Each statement below is calling a constructor with a single
        //   parameter, in this case being a function for calling the gamepad button. This does not set the
        //   current gamepad state only in the ToggleButtonWatcher; rather, it sets the ability for the
        //   ToggleButtonWatcher to invoke the gamepad values.
        watch_gamepad1_buttonY = ToggleButtonWatcher {gamepad1.y}
        watch_gamepad1_dpadDown = ToggleButtonWatcher {gamepad1.dpad_down}
        watch_gamepad1_dpadUp = ToggleButtonWatcher {gamepad1.dpad_up}
        watch_gamepad1_buttonA = ToggleButtonWatcher { gamepad1.a }
        watch_gamepad1_buttonB = ToggleButtonWatcher { gamepad1.b }
        watch_gamepad2_leftStickButton = ToggleButtonWatcher { gamepad2.left_stick_button }
        watch_gamepad2_rightStickButton = ToggleButtonWatcher { gamepad2.right_stick_button }
        watch_gamepad2_buttonA = ToggleButtonWatcher { gamepad2.a }
        watch_gamepad2_buttonB = ToggleButtonWatcher { gamepad2.b && !gamepad2.start }
        watch_gamepad2_dpadLeft = ToggleButtonWatcher { gamepad2.dpad_left }
        watch_gamepad2_dpadRight = ToggleButtonWatcher { gamepad2.dpad_right }
        watch_gamepad1_leftBumper = ToggleButtonWatcher { gamepad1.left_bumper }
        watch_gamepad1_rightBumper = ToggleButtonWatcher { gamepad1.right_bumper }

        intakeLiftBaseline = 0

//        container = VisionFactory.createOpenCv(VisionFactory.CameraType.WEBCAM, hardwareMap, IntakeRingViewingPipeline())
        container?.pipeline?.shouldKeepTracking = true
    }

    override fun start() {
        container?.pipeline?.tracking = true
        robot.intakeSystem.intakeStage1Trigger = { container?.pipeline?.ringVisibleOutsideIntake == true }
//        robot.intakeSystem.intakeStage2Trigger = { container.pipeline.ringVisibleInsideIntake }
        robot.intakeSystem.usePotentiometer = true
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
        if (canControlIntakeOrientation && gamepad1.a) reverse = false
        else if (canControlIntakeOrientation && gamepad1.b) reverse = true

        // reverse robot orientation using toggle feature in addition to static buttons
        // if y button is pressed, make reverse variable opposite of what it is now
        //   then wait until y is released before letting it change the variable again
        if (canControlIntakeOrientation && watch_gamepad1_buttonY.invoke()) reverse = !reverse

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

        if (gamepad2.left_bumper) {
            // Let's provide options for grabbing the intake
            when {
                gamepad2.a -> robot.wobbleGrabber.move(pivot = WobbleGrabber.PivotPosition.GRAB)
                gamepad2.x -> robot.wobbleGrabber.move(grab = WobbleGrabber.GrabPosition.GRAB)
                gamepad2.b -> robot.wobbleGrabber.move(pivot = WobbleGrabber.PivotPosition.PERPENDICULAR)
                gamepad2.y -> robot.wobbleGrabber.move(grab = WobbleGrabber.GrabPosition.MID_GRAB)
            }
        } else if (gamepad2.right_bumper) {
            // Let's provide options for releasing the intake in yeet position
            when {
                gamepad2.a -> robot.wobbleGrabber.move(pivot = WobbleGrabber.PivotPosition.YEET)
                gamepad2.x -> robot.wobbleGrabber.move(pivot = WobbleGrabber.PivotPosition.OVER_WALL)
                gamepad2.b -> robot.wobbleGrabber.move(pivot = WobbleGrabber.PivotPosition.STORAGE)
                gamepad2.y -> robot.wobbleGrabber.move(grab = WobbleGrabber.GrabPosition.STORAGE)
            }
        }

        when {
            (watch_gamepad2_dpadLeft()  && gamepad2.start) || (watch_gamepad1_leftBumper() && gamepad1.start) -> robot.wobbleGrabber.prevState()
            (watch_gamepad2_dpadRight() && gamepad2.start) || (watch_gamepad1_rightBumper() && gamepad1.start) -> robot.wobbleGrabber.nextState()
        }
        robot.wobbleGrabber.update()



    }

    // set PID control coefficients for auto-intake raise in controlIntakeMechanism()
    // this should eventually be moved
    var maxTriggerSpeed = 1.0
    var didActivateMax = false
    var intakeRaiseInput = 0.0
    private fun controlIntakeMechanism() {
        val input = (
                if (gamepad2.left_stick_y.absoluteValue > 0.05)
                    gamepad2.left_stick_y
                else if (gamepad1.left_trigger > 0.05) {
                    gamepad1.left_trigger
                }
                else if (gamepad1.right_trigger > 0.05) {
                    -gamepad1.right_trigger
                }
                else if (robot.intakeSystem.intakeArmState == FullIntakeSystem.IntakeArmState.IDLE)
                    0.0f
                else
                    null
                )?.toDouble()

        if (gamepad1.left_trigger > 0.05 || gamepad1.right_trigger > 0.05) {
            if (input?.absoluteValue == 1.0 && !didActivateMax) {
                didActivateMax = true
                robot.intakeSystem.moveIntake(if (input == -1.0) FullIntakeSystem.IntakePosition.SCORE else FullIntakeSystem.IntakePosition.GROUND)
            } else {
                if (!didActivateMax && input != null)
                    robot.intakeSystem.manualMoveIntake(input)
            }
        } else {
            didActivateMax = false
            if (input != null) robot.intakeSystem.manualMoveIntake(input)
        }


//        robot.intakeSystem.manualRingMotor(
//                when {
//                    gamepad2.left_trigger  > 0.05 -> -gamepad2.left_trigger.toDouble()
//                    gamepad2.right_trigger > 0.05 -> gamepad2.right_trigger.toDouble()
//                    else                          -> 0.0
//                }
//        )
        val speedInput = gamepad2.right_trigger.toDouble()
        if (speedInput > 0.05 && maxTriggerSpeed < speedInput) {
            maxTriggerSpeed = speedInput
            robot.intakeSystem.desiredRingIntakePower = speedInput
        }
        else if (speedInput < 0.05) {
            maxTriggerSpeed = 0.0
        }

        if(!(gamepad2.left_bumper || gamepad2.right_bumper)) {
            when {
                watch_gamepad2_buttonA.invoke() || (watch_gamepad1_buttonA() && !canControlIntakeOrientation) -> robot.intakeSystem.ringIntakeState = FullIntakeSystem.RingIntakeState.COLLECT_STAGE_1
                watch_gamepad2_buttonB.invoke() || (watch_gamepad1_buttonB() && !canControlIntakeOrientation) -> robot.intakeSystem.ringIntakeState = FullIntakeSystem.RingIntakeState.IDLE
//                gamepad2.x || (gamepad1.x && !canControlDrivetrainOrientation) -> robot.intakeSystem.ringDropOnWobble()
                gamepad2.y || (gamepad1.y && !canControlIntakeOrientation) -> robot.intakeSystem.rejectRing()
                gamepad2.dpad_up -> robot.intakeSystem.moveIntake(FullIntakeSystem.IntakePosition.SCORE)
                gamepad2.dpad_down -> robot.intakeSystem.moveIntake(FullIntakeSystem.IntakePosition.GROUND)
                gamepad2.left_stick_button && gamepad2.start -> robot.intakeSystem.resetZero()
                watch_gamepad2_rightStickButton.invoke() -> robot.intakeSystem.shouldUseLambdaActivation =
                        !robot.intakeSystem.shouldUseLambdaActivation
            }
        }

        if (watch_gamepad2_leftStickButton() && !gamepad2.start) robot.intakeSystem.shouldTempHold = !robot.intakeSystem.shouldTempHold

//        if (watch_gamepad2_dpadLeft.invoke()) robot.intakeSystem.usePotentiometer = !robot.intakeSystem.usePotentiometer

//        when {
//            gamepad2.dpad_up -> robot.ringDropper.pivot(RingDropper.DropperPosition.HOLD_RING)
//            gamepad2.dpad_down -> robot.ringDropper.pivot(RingDropper.DropperPosition.INTAKE)
//        }

        robot.intakeSystem.update()
    }

    private fun controlMusic() {
        // if gamepad2 right stick button is toggled, toggle music
//        if (watch_gamepad2_rightStickButton.invoke()) {
//            if (musicPlayer.isPlaying()) musicPlayer.pause()
//            else musicPlayer.play()
//        }

        robot.blinkinController.update(
                when (robot.intakeSystem.intakeArmState) {
                    FullIntakeSystem.IntakeArmState.RAISE -> RevBlinkinLedDriver.BlinkinPattern.YELLOW
                    FullIntakeSystem.IntakeArmState.TEMP_HOLD -> RevBlinkinLedDriver.BlinkinPattern.BLUE
                    FullIntakeSystem.IntakeArmState.IDLE -> RevBlinkinLedDriver.BlinkinPattern.RED
                }  .takeUnless { it == robot.blinkinController.currentState }
        )
    }

    private fun controlTelemetry() {
        telemetry.addData("IntakeLiftMotor pos", robot.intakeLiftMotor.currentPosition)
        telemetry.addData("IntakeLiftMotor pwr", robot.intakeLiftMotor.power)
        telemetry.addLine()
        telemetry.addData("Potentiometer pos", robot.liftPotentiometer.voltage)
        telemetry.addData("Use potentiometer", robot.intakeSystem.usePotentiometer)
        telemetry.addLine()
        telemetry.addData("Raise last deriv", robot.intakeSystem.raiseLastDeriv)
        telemetry.addData("Raise integral sum", robot.intakeSystem.raiseIntegralSum)
        telemetry.addData("Raise last error", robot.intakeSystem.raiseLastError)
        telemetry.addLine()
        telemetry.addData("Intake arm state", robot.intakeSystem.intakeArmState)
        telemetry.addData("Intake arm target", robot.intakeSystem.intakeArmTarget)
        telemetry.addData("Intake ring state", robot.intakeSystem.ringIntakeState)
        telemetry.addData("Intake ring state (prev)", robot.intakeSystem.previousRingIntakeState)
        telemetry.addData("Intake ring state (next)", robot.intakeSystem.nextRingIntakeState)
        telemetry.addLine()
        telemetry.addData("Intake manual raise input", intakeRaiseInput)
        telemetry.addData("Intake did activate max", didActivateMax)
        telemetry.addData("Intake should do temp hold", robot.intakeSystem.shouldTempHold)
        telemetry.addLine()
        telemetry.addData("Intake ring speed", robot.intakeSystem.desiredRingIntakePower)
        telemetry.addData("Intake ring fully in", robot.intakeSystem.ringFullyInIntake)
        telemetry.addLine()
        telemetry.addData("Ring visible out", container?.pipeline?.ringVisibleOutsideIntake)
        telemetry.addData("Num qualify out", container?.pipeline?.numSuccessfulOutsideIntake)
        telemetry.addLine()
        telemetry.addData("Ring visible in", container?.pipeline?.ringVisibleInsideIntake)
        telemetry.addData("Num qualify in", container?.pipeline?.numSuccessfulInsideIntake)
        telemetry.addLine()
        telemetry.addData("Ring fully in", robot.intakeSystem.ringFullyInIntake)
        telemetry.addData("Should lambda activate", robot.intakeSystem.shouldUseLambdaActivation)
        telemetry.addLine()
        telemetry.addData("Wobble state", robot.wobbleGrabber.state)
        telemetry.addData("Wobble state (prev)", robot.wobbleGrabber.state)
        telemetry.addData("Wobble state (next)", robot.wobbleGrabber.state)
        telemetry.addLine()
        telemetry.addData("Cycle rate", System.currentTimeMillis()-lastCycleTime)
        lastCycleTime = System.currentTimeMillis()
    }

    // functionality is explained throughout opmode; allows for encapsulation of button presses
    //   that toggle changes in variables
    // constructor parameter is a function (no parameters, returns Boolean)
}



