package org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.TelemetryMenu
import kotlin.properties.Delegates
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KProperty

class DelegatedTelemetryMenu constructor(private val telemetry: Telemetry) : TelemetryMenu {
    private val list = emptyList<MenuItemDelegate<*>>().toMutableList()
    private var current : Int by Delegates.vetoable(0) {
        _, _, newValue ->
        newValue in 0 until list.size
    }

    fun refresh() {
        list.forEach {
            telemetry.addData((if (it === list[current]) "-> " else "") + it.description,
                    (if (it.canIterateBackward()) " << " else "")
                            + it
                            + (if (it.canIterateForward()) " >> " else ""))
        }
        telemetry.update()
    }

    fun add(menuItem: MenuItemDelegate<*>) {
        list.add(menuItem)
        refresh()
    }

    override fun nextItem() {
        current++
        refresh()
    }
    override fun previousItem() {
        current--
        refresh()
    }
    override fun iterateForward() {
        list[current].iterateForward()
        refresh()
    }
    override fun iterateBackward() {
        list[current].iterateBackward()
        refresh()
    }

}

abstract class MenuItemDelegate<T>(
                                   val description : String,
                                   var value: T) : ReadWriteProperty<Any?, T> {
    override fun getValue(thisRef: Any?, property: KProperty<*>): T {
        return value
    }
    override fun toString(): String = value.toString()
    abstract fun iterateForward()
    abstract fun iterateBackward()
    abstract fun canIterateForward(): Boolean
    abstract fun canIterateBackward(): Boolean

    infix fun with(menu: DelegatedTelemetryMenu): MenuItemDelegate<T>
    {
        menu.add(this)
        return this
    }
}

class MenuItemIntDelegate(
                          description: String,
                          startingValue: Int,
                          private val lowerLimit: Int,
                          private val upperLimit: Int,
                          private val incrementBy: Int = 1)
    : MenuItemDelegate<Int>(
        description,
        if (startingValue in lowerLimit..upperLimit) startingValue else lowerLimit) {

    override fun getValue(thisRef: Any?, property: KProperty<*>): Int {
        return value
    }

    override fun setValue(thisRef: Any?, property: KProperty<*>, value: Int) {
        if (value in lowerLimit..upperLimit) this.value = value
    }

    override fun iterateForward() {
        if (canIterateForward()) value+=incrementBy
    }

    override fun iterateBackward() {
        if (canIterateBackward()) value-=incrementBy
    }

    override fun canIterateForward(): Boolean = value+incrementBy <= upperLimit

    override fun canIterateBackward(): Boolean = value-incrementBy >= lowerLimit

}

class MenuItemBooleanDelegate(
                              description: String,
                              startingValue: Boolean)
    : MenuItemDelegate<Boolean>(description, startingValue) {

    override fun iterateForward() {
        if (canIterateForward()) value = true
    }

    override fun iterateBackward() {
        if (canIterateBackward()) value = false
    }

    override fun canIterateForward(): Boolean = !value // aka "is value false"

    override fun canIterateBackward(): Boolean = value // aka "is value true"

    override fun setValue(thisRef: Any?, property: KProperty<*>, value: Boolean) {
        super.value = value
    }

}
class MenuItemEnumDelegate<T>(
                              description: String,
                              private vararg val values: T)
    : MenuItemDelegate<T>(description, values[0]) {
    override fun iterateForward() {
        if (canIterateForward()) value = values[values.indexOf(value)+1]
    }

    override fun iterateBackward() {
        if (canIterateBackward()) value = values[values.indexOf(value)-1]
    }

    override fun canIterateForward(): Boolean = values.indexOf(value) < values.size - 1

    override fun canIterateBackward(): Boolean = values.indexOf(value) > 0

    override fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
        if (values.contains(value)) this.value = value
    }
}