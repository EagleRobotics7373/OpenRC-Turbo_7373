package org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos

import com.qualcomm.robotcore.hardware.Servo

class RingDropper
constructor (private val servo: Servo)
{

    enum class DropperPosition(val position: Double) {
        ONTO_WOBBLE(0.90),
        INTAKE(0.73),
        HOLD_RING(0.62)
    }

    fun pivot(loc: DropperPosition) { servo.position = loc.position }

}