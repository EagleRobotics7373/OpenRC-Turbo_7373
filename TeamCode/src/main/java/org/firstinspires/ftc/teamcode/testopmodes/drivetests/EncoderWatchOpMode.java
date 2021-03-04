package org.firstinspires.ftc.teamcode.testopmodes.drivetests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.functions.FunctionalExtensionsKt;
import org.firstinspires.ftc.teamcode.library.functions.roadrunnersupport.Encoder;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BaseRobot;
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.OdometryModule;

import static org.firstinspires.ftc.teamcode.library.robot.robotcore.RobotProvider.providePresetRobot;

@TeleOp(name="Encoder Watch OpMode", group="Test")
public class EncoderWatchOpMode extends OpMode {
    BaseRobot robot;
    MultipleTelemetry telem;
    @Override
    public void init() {
        robot = providePresetRobot(hardwareMap);
        robot.holonomic.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        telem.addData("flm cp", robot.frontLeftMotor.getCurrentPosition());
        telem.addData("frm cp", robot.frontRightMotor.getCurrentPosition());
        telem.addData("blm cp", robot.backLeftMotor.getCurrentPosition());
        telem.addData("brm cp", robot.backRightMotor.getCurrentPosition());

        addOdometryData(robot.getLeftOdometryModule(), "Left");
        addOdometryData(robot.getRightOdometryModule(), "Right");
        addOdometryData(robot.getRearOdometryModule(), "Rear");

        telem.addData("imu position (deg)", FunctionalExtensionsKt.toDegrees(robot.getHolonomicRR().getExternalHeading()));

        telem.update();

        robot.holonomic.runWithoutEncoderVectored(
                gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                0.0
        );

    }

    private void addOdometryData(Encoder module, String name) {
        if (module != null) {
            int pos = module.getCurrentPosition();
            telem.addData("odo" + name + " cp raw", pos);
            telem.addData("oxo" + name + " cp cm", module.normalizePosition(pos, DistanceUnit.CM));
            telem.addData("oxo" + name + " cp in", module.normalizePosition(pos, DistanceUnit.INCH));
        }
    }

}
