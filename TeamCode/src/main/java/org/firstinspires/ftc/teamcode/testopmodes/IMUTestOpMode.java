package org.firstinspires.ftc.teamcode.testopmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController;

@TeleOp (name= "IMU Test OpMode", group="Test")
public class IMUTestOpMode extends OpMode {
    IMUController imuController;

    @Override
    public void init() {
        imuController = new IMUController(hardwareMap, AxesOrder.ZYX);
    }

    @Override
    public void loop() {
        Orientation ao = imuController.imuA.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("first", ao.firstAngle);
        telemetry.addData("second", ao.secondAngle);
        telemetry.addData("third", ao.thirdAngle);
        telemetry.update();

    }
}
