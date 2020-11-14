package org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos

import com.qualcomm.hardware.rev.RevBlinkinLedDriver

class BlinkinController(private val blinkin: RevBlinkinLedDriver, private val changeStateDelta: Long) {

    private var lastChange: Long = System.currentTimeMillis()

    var currentState: RevBlinkinLedDriver.BlinkinPattern? = null
    private set(value) {
        if (value == null) return
        field = value
        lastChange = System.currentTimeMillis()
        blinkin.setPattern(value)
    }

    private val timeSinceLastChange: Long get() = System.currentTimeMillis() - lastChange

    private var desiredState: RevBlinkinLedDriver.BlinkinPattern? = null

    fun update(state: RevBlinkinLedDriver.BlinkinPattern? = null) {
        if (state != null) {
            desiredState = state
        }

        if (desiredState != null && timeSinceLastChange >= changeStateDelta) {
            currentState = desiredState
            desiredState = null
        }
    }
}