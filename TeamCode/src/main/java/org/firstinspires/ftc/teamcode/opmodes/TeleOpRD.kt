package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.functions.rangeBuffer
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot

@TeleOp(name="TeleOp RD", group="basic")
open class TeleOpRD : OpMode() {
    lateinit var robot : BasicRobot
    var playingMusic = false

    override fun init() {
        robot = BasicRobot(hardwareMap)
    }

    override fun loop() {
        controlDrivetrain()
        controlFoundationGrabbers()
    }

    protected open fun controlDrivetrain() {
        val x = -gamepad1.left_stick_x.toDouble().rangeBuffer(-0.1, 0.1, 0.0)
        val y = gamepad1.left_stick_y.toDouble().rangeBuffer(-0.1, 0.1, 0.0)
        val z = -gamepad1.right_stick_x.toDouble().rangeBuffer(-0.1, 0.1, 0.0)

        robot.holonomic.runWithoutEncoderVectored(x, y, z, 0.0)
    }

    private fun controlFoundationGrabbers() {
        if (gamepad2.dpad_down) robot.foundationGrabbers.lock()
        else if (gamepad2.dpad_up) robot.foundationGrabbers.unlock()
    }

    private fun controlMusic() {
    }
}



