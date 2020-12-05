package org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos

import com.qualcomm.robotcore.hardware.Servo

class RingTapper(private val servo: Servo) {

    enum class Position(val pos: Double) {
        TAP(0.45), STORAGE(0.6)
    }

    fun move(to: Position) {
        this.servo.position = to.pos
    }

}