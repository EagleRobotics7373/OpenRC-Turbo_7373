package org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos

import com.qualcomm.robotcore.hardware.Servo

class RingTapper(private val servo: Servo, private val storedPos: Double, private val tapPos: Double) {

    enum class Position {
        TAP, STORAGE
    }

    fun move(to: Position) {
        this.servo.position = when(to) {
            Position.TAP ->  tapPos
            Position.STORAGE -> storedPos
        }
    }

}