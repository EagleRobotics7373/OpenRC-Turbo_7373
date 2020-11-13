package org.firstinspires.ftc.teamcode.testopmodes.servotests

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.library.functions.ToggleButtonWatcher
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin.*

@TeleOp
class SelectableServoTest : OpMode() {
    lateinit var menu: ReflectiveTelemetryMenu
    lateinit var servo: Servo
    lateinit var servos : Array<Servo>

    var useGamepadTrigger = true
    var enable = true
    var tenths = 0
    var hundredths = 0

    lateinit var dpadUpWatch : ToggleButtonWatcher
    lateinit var dpadDownWatch : ToggleButtonWatcher
    lateinit var dpadLeftWatch : ToggleButtonWatcher
    lateinit var dpadRightWatch : ToggleButtonWatcher

    override fun init() {
        servos = hardwareMap.getAll(Servo::class.java).toTypedArray()
        if (servos.isNullOrEmpty()) throw(IllegalStateException("There are no registered servo devices!"))
        servo = servos.first()

        menu = ReflectiveTelemetryMenu(
                telemetry,
                ReflectiveMenuItemEnum("servo", ::servo, *servos, toStringMethod = {hardwareMap.getNamesOf(it).first()}),
                ReflectiveMenuItemInteger("0.x0", ::tenths, 0, 9, 1),
                ReflectiveMenuItemInteger("0.0x", ::hundredths, 0, 9, 1),
                ReflectiveMenuItemBoolean("use gamepad trigger", ::useGamepadTrigger),
                ReflectiveMenuItemFeedback("servo position") {servo.position.toString()},
                ReflectiveMenuItemFeedback("enable") {enable.toString()}
        )

        dpadUpWatch = ToggleButtonWatcher {gamepad1.dpad_up}
        dpadDownWatch = ToggleButtonWatcher {gamepad1.dpad_down}
        dpadLeftWatch = ToggleButtonWatcher {gamepad1.dpad_left}
        dpadRightWatch = ToggleButtonWatcher {gamepad1.dpad_right}


    }

    override fun loop() {
            when {
                dpadUpWatch.invoke()    -> menu.previousItem()
                dpadDownWatch.invoke()  -> menu.nextItem()
                dpadLeftWatch.invoke()  -> menu.iterateBackward()
                dpadRightWatch.invoke() -> menu.iterateForward()
                else -> menu.refresh()
            }
        if (enable) {
            servo.position =
                    if (useGamepadTrigger) gamepad1.left_trigger.toDouble()
                    else tenths * 0.10 + hundredths * 0.01
        }
        if (gamepad1.a) enable = true
        else if (gamepad1.b) enable = false
    }
}