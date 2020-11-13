package org.firstinspires.ftc.teamcode.library.functions

class ToggleButtonWatcher(private val getMethod: ()->Boolean) {
    private var lastState : Boolean = getMethod()
    operator fun invoke(): Boolean {
        if (getMethod()) {
            if (!lastState) {
                lastState = true
                return true
            }
        } else lastState = false
        return false
    }
}