package org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos

import com.qualcomm.robotcore.hardware.Servo

class WobbleGrabber
constructor (private val pivotServo : Servo,
             private val grabServo  : Servo)
{
    fun pivotDown()     { pivotServo.position = 0.93 }
    fun pivotVertical() { pivotServo.position = 0.60 }
    fun pivotBack()     { pivotServo.position = 0.23 }

    fun grab()          { grabServo.position = 0.00 }
    fun midGrab()       { grabServo.position = 0.27 }
    fun releaseGrab()   { grabServo.position = 0.60 }
}