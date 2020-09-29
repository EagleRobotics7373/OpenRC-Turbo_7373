package org.firstinspires.ftc.teamcode.testopmodes.visiontests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.legacyconfig.MisumiRobot;
import org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory;
import org.firstinspires.ftc.teamcode.library.vision.base.VuforiaController;

import static org.firstinspires.ftc.teamcode.library.vision.base.VisionFactory.createVuforia;
@Autonomous(name="Modular Vuforia Test", group="Test")
public class ModularVuforiaTest extends LinearOpMode {
    MisumiRobot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MisumiRobot(hardwareMap);
        VuforiaLocalizer vuforia = createVuforia(VisionFactory.CameraType.PHONE_REAR, hardwareMap);
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
