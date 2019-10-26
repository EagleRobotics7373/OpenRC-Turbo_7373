package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.library.functions.*
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot
import kotlin.math.absoluteValue
import kotlin.math.pow

@TeleOp(name="TeleOp RD", group="basic")
open class TeleOpRD : OpMode() {
    lateinit var robot : BasicRobot
    val musicPlayer = ExtDirMusicPlayer(ExtMusicFile.UNITY)
    var playingMusic = false
    var elapsedTime : ElapsedTime? = null
    var last = 0.0
    var current = 0.0

    override fun init() {
        robot = BasicRobot(hardwareMap)
    }

    override fun loop() {
        controlDrivetrain()
        controlFoundationGrabbers()
        controlIntakeMechanism()
        controlTelemetry()
        controlMusic()
    }

    override fun stop() {
        super.stop()
        musicPlayer.stop()
    }

    protected open fun controlDrivetrain() {
        val x = gamepad1.left_stick_x.toDouble().rangeBuffer(-0.1, 0.1, 0.0)
        val y = -gamepad1.left_stick_y.toDouble().rangeBuffer(-0.1, 0.1, 0.0)
        val z = gamepad1.right_stick_x.toDouble().rangeBuffer(-0.1, 0.1, 0.0)


//        robot.holonomic.runWithoutEncoderVectored(x, y, z, 0.0)
        robot.holonomic.runWithoutEncoder(x, y, z)
    }

    private fun controlFoundationGrabbers() {
        if (gamepad2.dpad_down) robot.foundationGrabbers.lock()
        else if (gamepad2.dpad_up) robot.foundationGrabbers.unlock()
    }

    private var intakeP = 4.0
    private var intakeI = 0.000
    private var intakeD = 0.20
    private val voltageTarget = 1.257
    private var lastRuntime = -1.0
    private var d = 0.0
    private var lastError = 0.0
    private var errorSum = 0.0
    private fun controlIntakeMechanism() {
        val input =
                if (gamepad2.b) {
                    //use pid loop
                    val error = voltageTarget-robot.intakePivotPotentiometer.voltage
                    if (lastRuntime==-1.0) lastRuntime = runtime
                    errorSum += error*runtime-lastRuntime

                    d = error-lastError
                    lastRuntime = runtime
                    lastError = error

                    -(intakeP * error + intakeI * errorSum + intakeD * d)
                }
                else gamepad2.left_stick_y.toDouble()
        robot.intakePivotMotor.power =
                when {
                    input > 0.0 -> input+0.02
                    else -> 0.02 - -input*0.04
                }/*.rangeBuffer(-0.10, 0.10, 0.0)*/

//        if (gamepad2.left_trigger > 0.10)
//            robot.intakeBlockManipulator.power = -gamepad2.left_trigger.toDouble()
//        else if (gamepad2.right_trigger > 0.10)
//            robot.intakeBlockManipulator.power = gamepad2.right_trigger.toDouble()
//        else
//            robot.intakeBlockManipulator.power = 0.0

        robot.intakeBlockManipulator.power = -gamepad2.right_stick_y.toDouble()
    }

    private var musicButtonHeld = false
    private fun controlMusic() {
        if (gamepad1.left_stick_button) musicPlayer.play()
        else if (gamepad1.right_stick_button) musicPlayer.pause()


    }

    private fun controlTelemetry() {
        telemetry.addData("Intake manipulator power", robot.intakeBlockManipulator.power)
        telemetry.addData("Intake pivot power", robot.intakePivotMotor.power)
//        telemetry.addData("Potentiometer voltage", robot.intakePivotPotentiometer.voltage)
//        telemetry.addData("Potentiometer max v", robot.intakePivotPotentiometer.maxVoltage)
//        telemetry.addData("Hue in range?", robot.intakeBlockCSensor.rhue in 0.32..0.41)
//        telemetry.addData("hue", robot.intakeBlockCSensor.rhue)
//        telemetry.addData("DS distance", robot.intakeBlockDSensor.getDistance(DistanceUnit.MM))
        telemetry.update()
    }
    fun setCoeffs(p: Double, i: Double, d: Double) {
        intakeP = p
        intakeI = i
        intakeD = d
    }
}



