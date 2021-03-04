package org.firstinspires.ftc.teamcode.library.robot.systems.intakegen3

import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.library.functions.rhue
import org.firstinspires.ftc.teamcode.library.functions.rsaturation

class MagazineRingSensor(private val colorSensor: ColorSensor, private val distanceSensor: DistanceSensor) {
    val ringPresent: Boolean
    get() = (colorSensor.rsaturation > 0.55
            && distanceSensor.getDistance(DistanceUnit.CM) < 6)
}
