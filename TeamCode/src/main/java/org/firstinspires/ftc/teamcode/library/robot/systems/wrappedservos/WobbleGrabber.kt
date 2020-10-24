package org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos

import com.qualcomm.robotcore.hardware.Servo

class WobbleGrabber
constructor (private val pivotServo : Servo,
             private val grabServo  : Servo)
{

    enum class PivotPosition(val position: Double) {
        GRAB(0.99),
        PERPENDICULAR(0.80),
        OVER_WALL(0.55),
        VERTICAL(0.40),
        YEET(0.30),
        STORAGE(0.23)
    }

    fun pivot(loc: PivotPosition) { pivotServo.position = loc.position }

    enum class GrabPosition(val position: Double) {
        GRAB(0.01),
        MID_GRAB(0.33),
        STORAGE(0.78)
    }

    fun grab(loc: GrabPosition) { grabServo.position = loc.position }
}