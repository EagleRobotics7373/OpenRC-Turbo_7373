package org.firstinspires.ftc.teamcode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot;
import org.firstinspires.ftc.teamcode.library.vision.skystone.VisionInitializer;
import org.firstinspires.ftc.teamcode.library.vision.skystone.VuforiaController;

import static org.firstinspires.ftc.teamcode.library.vision.skystone.VisionInitializer.createVuforia;
@Autonomous(name="Modular Vuforia Test", group="Test")
public class ModularVuforiaTest extends LinearOpMode {
    BasicRobot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BasicRobot(hardwareMap);
        VuforiaLocalizer vuforia = createVuforia(VisionInitializer.CameraType.PHONE_REAR, hardwareMap);
        VuforiaController vuforiaController = new VuforiaController(vuforia,telemetry);

        waitForStart();
        vuforiaController.activate();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                vuforiaController.analyzeVuforiaResult();
            }
        }

        vuforiaController.deactivate();
    }

}
