package org.firstinspires.ftc.teamcode.opmodes.util

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcontroller.internal.NetworkUtil
import org.firstinspires.ftc.teamcode.opmodes.gen1b.OpModeConfig

@TeleOp(name = "Wi-Fi Provisioning Tool", group = "Utilities")
class WiFiProvisioner: LinearOpMode() {

    val config = OpModeConfig(telemetry)
    var teamNumber = config.custom("Team number", 7373, 11364)
    var deviceType = config.custom("Device type", "RC", "DS")
    var groupLetter = config.custom("Group letter", null, *(CharRange('A', 'Z').map {it}.toTypedArray()))
    val networkName: String get() = "$teamNumber-$groupLetter-$deviceType"
    val password   : String get() = "eaglerobotics_$teamNumber"

    override fun runOpMode() {

        config.update()
        while (!isStarted && !isStopRequested) {
            when {
                gamepad1.dpad_up -> config.update(prevItem = true)
                gamepad1.dpad_down -> config.update(nextItem = true)
                gamepad1.dpad_left -> config.update(iterBack = true)
                gamepad1.dpad_right -> config.update(iterFw = true)
            }
            while(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right);
        }

        NetworkUtil.setNetworkSettings(networkName, password)
        telemetry.addLine("Stored new network settings")
        telemetry.addData("Network name", networkName)
        telemetry.addData("Password", password)
        sleep(1000)
    }
}