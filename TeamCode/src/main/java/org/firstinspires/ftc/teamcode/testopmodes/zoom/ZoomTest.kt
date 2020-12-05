package org.firstinspires.ftc.teamcode.testopmodes.zoom

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp(group="zoom")
class ZoomTest: LinearOpMode() {
    override fun runOpMode() {
        val via = hardwareMap.get(DcMotor::class.java, "via")
        val thru = hardwareMap.get(DcMotor::class.java, "thru")

        waitForStart()

        while (opModeIsActive()) {
            via.power = gamepad1.left_stick_y.toDouble()
            thru.power = gamepad1.right_stick_y.toDouble()
        }
    }
}