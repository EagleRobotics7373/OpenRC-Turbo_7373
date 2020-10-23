package org.firstinspires.ftc.teamcode.opmodes.gen2

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin.*

class OpModeConfig(var telemetry: Telemetry) {

    val menu = DelegatedTelemetryMenu(telemetry)

    fun int(description: String, startingValue: Int, progression: IntProgression): MenuItemDelegate<Int> {
        return MenuItemIntDelegate(description, startingValue, progression.first, progression.last, progression.step) with menu
    }

    fun boolean(description: String, startingValue: Boolean): MenuItemDelegate<Boolean> {
        return MenuItemBooleanDelegate(description, startingValue) with menu
    }

    fun <T> custom(description: String, vararg values: T): MenuItemDelegate<T> {
        return MenuItemEnumDelegate<T>(description, *values) with menu
    }

    fun update(prevItem: Boolean = false,
               nextItem: Boolean = false,
               iterBack: Boolean = false,
               iterFw  : Boolean = false) {
        when {
            prevItem -> menu.previousItem()
            nextItem -> menu.nextItem()
            iterBack -> menu.iterateBackward()
            iterFw   -> menu.iterateForward()
            else     -> menu.refresh()
        }
    }
}