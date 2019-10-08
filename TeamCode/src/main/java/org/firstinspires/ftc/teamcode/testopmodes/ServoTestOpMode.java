package org.firstinspires.ftc.teamcode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.library.functions.MathExtensionsKt;

@TeleOp(name="CRServo Test", group="Test")
public class ServoTestOpMode extends OpMode {
    CRServo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(CRServo.class, "crservo");
    }

    @Override
    public void loop() {
        servo.setPower(MathExtensionsKt.rangeClip(gamepad1.left_stick_y, -0.9, 0.9));
        telemetry.addData("power", gamepad1.left_stick_y);
    }
}
