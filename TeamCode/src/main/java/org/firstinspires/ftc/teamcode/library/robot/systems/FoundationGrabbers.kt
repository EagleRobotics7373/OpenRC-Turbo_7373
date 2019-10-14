package org.firstinspires.ftc.teamcode.library.robot.systems

import com.qualcomm.robotcore.hardware.Servo

class FoundationGrabbers(
        private val leftGrabberServo: Servo,
        private val rightGrabberServo:Servo) {

    fun unlock() {
        leftGrabberServo.position  = 1.0
        rightGrabberServo.position = 1.0
    }

    fun lock() {
        leftGrabberServo.position  = 0.47
        rightGrabberServo.position = 0.50
    }
}