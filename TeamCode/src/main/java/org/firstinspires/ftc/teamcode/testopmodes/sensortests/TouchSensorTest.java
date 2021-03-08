package org.firstinspires.ftc.teamcode.testopmodes.sensortests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(group="Test")
public class TouchSensorTest extends OpMode {
    DigitalChannel touchSensor;

    @Override
    public void init() {
        touchSensor = hardwareMap.get(DigitalChannel.class, "intakeTouchSensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
    }

    @Override
    public void loop() {
        telemetry.addData("pressed", touchSensor.getState());
    }
}
