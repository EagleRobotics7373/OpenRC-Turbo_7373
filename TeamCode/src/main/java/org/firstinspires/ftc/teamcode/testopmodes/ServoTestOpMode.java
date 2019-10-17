package org.firstinspires.ftc.teamcode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.library.functions.MathExtensionsKt;

@TeleOp(name="Servo Test", group="Test")
public class ServoTestOpMode extends OpMode {
    Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
    }

    @Override
    public void loop() {
        servo.setPosition(gamepad1.left_trigger);
        telemetry.addData("power", gamepad1.left_trigger);
    }
}
