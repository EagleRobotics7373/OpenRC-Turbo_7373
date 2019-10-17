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
    }

    override fun stop() {
        super.stop()
        musicPlayer.stop()
    }

    protected open fun controlDrivetrain() {
        val x = -gamepad1.left_stick_x.toDouble().rangeBuffer(-0.1, 0.1, 0.0).pow(3)
        val y = gamepad1.left_stick_y.toDouble().rangeBuffer(-0.1, 0.1, 0.0).pow(3)
        val z = -gamepad1.right_stick_x.toDouble().rangeBuffer(-0.1, 0.1, 0.0).pow(3)

        robot.holonomic.runWithoutEncoderVectored(x, y, z, 0.0)
    }

    private fun controlFoundationGrabbers() {
        if (gamepad2.dpad_down) robot.foundationGrabbers.lock()
        else if (gamepad2.dpad_up) robot.foundationGrabbers.unlock()
    }

    private fun controlIntakeMechanism() {
        robot.intakePivotMotor.power =
                when {
                    gamepad2.left_stick_y.toDouble() > 0.0 -> gamepad2.left_stick_y.toDouble()+0.02
                    else -> 0.02 - -gamepad2.left_stick_y.toDouble()*0.04
                }/*.rangeBuffer(-0.10, 0.10, 0.0)*/

        if (gamepad2.left_trigger > 0.10)
            robot.intakeBlockManipulator.power = -gamepad2.left_trigger.toDouble()
        else if (gamepad2.right_trigger > 0.10)
            robot.intakeBlockManipulator.power = gamepad2.right_trigger.toDouble()
        else
            robot.intakeBlockManipulator.power = 0.0
    }

    private fun controlMusic() {
        if (gamepad1.y) musicPlayer.play()
        if (gamepad1.x) musicPlayer.pause()
    }

    private fun controlTelemetry() {
        telemetry.addData("Intake manipulator power", robot.intakeBlockManipulator.power)
        telemetry.addData("Intake pivot power", robot.intakePivotMotor.power)
        telemetry.addData("Potentiometer voltage", robot.intakePivotPotentiometer.voltage)
        telemetry.addData("Potentiometer max v", robot.intakePivotPotentiometer.maxVoltage)
        telemetry.addData("Hue in range?", robot.intakeBlockCSensor.rhue in 0.32..0.41)
        telemetry.addData("hue", robot.intakeBlockCSensor.rhue)
        telemetry.addData("DS distance", robot.intakeBlockDSensor.getDistance(DistanceUnit.MM))
        telemetry.update()
    }
}



