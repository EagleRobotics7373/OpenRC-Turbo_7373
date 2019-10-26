package org.firstinspires.ftc.teamcode.library.robot.systems

import com.qualcomm.robotcore.hardware.Servo

class FoundationGrabbers(
        private val grabberServo: Servo) {

    fun unlock() {
        grabberServo.position  = 0.10
    }

    fun lock() {
        grabberServo.position  = 0.6
    }
}