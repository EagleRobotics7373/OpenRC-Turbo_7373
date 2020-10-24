package org.firstinspires.ftc.teamcode.testopmodes.sensortests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.functions.FunctionalExtensionsKt;

@TeleOp(group="Test")
public class ColorSensorTest extends OpMode {
    ColorSensor left;
    ColorSensor right;

    DistanceSensor leftDs;

    @Override
    public void init() {
        left = hardwareMap.get(ColorSensor.class, "intakeColorSensor");
        leftDs = hardwareMap.get(DistanceSensor.class, "intakeColorSensor");
    }

    @Override
    public void loop() {
        telemetry.addData("left alpha", left.alpha());
        telemetry.addData("left red", left.red());
        telemetry.addData("left green", left.green());
        telemetry.addData("left blue", left.blue());
        telemetry.addData("left hue", FunctionalExtensionsKt.getRhue(left));
        telemetry.addData("left saturation", FunctionalExtensionsKt.getRsaturation(left));
        telemetry.addData("left value", FunctionalExtensionsKt.getRvalue(left));
        telemetry.addData("left distance (cm)", leftDs.getDistance(DistanceUnit.CM));



    }
}
