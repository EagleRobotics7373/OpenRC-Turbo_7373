package org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.kotlin

import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.properties.Delegates
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KProperty

class DelegatedTelemetryMenu constructor(private val telemetry: Telemetry) {
    private val list = emptyList<MenuItemDelegate<*>>().toMutableList()
    private var current : Int by Delegates.vetoable(0) {
        _, _, newValue ->
            newValue in 0 until list.size
    }
    private fun refresh() {
        list.forEach {
            telemetry.addData(if (it === list[current]) "-> " else "" + it.description,
                    if (it.canIterateBackward()) " << " else ""
                            + it
                            + if (it.canIterateForward()) " >> " else "")
        }
    }

    fun add(menuItem: MenuItemDelegate<*>) {
        list.add(menuItem)
        refresh()
    }

    fun nextItem() {
        current++
        refresh()
    }
    fun previousItem() {
        current--
        refresh()
    }
    fun iterateCurrentItemForward() {
        list[current].iterateForward()
    }
    fun iterateCurrentItemBackward() {
        list[current].iterateBackward()
    }

}

abstract class MenuItemDelegate<T>(menu : DelegatedTelemetryMenu,
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
}

class MenuItemIntDelegate(menu: DelegatedTelemetryMenu,
                          description: String,
                          startingValue: Int,
                          private val lowerLimit: Int,
                          private val upperLimit: Int,
                          private val incrementBy: Int = 1)
    : MenuItemDelegate<Int>(
        menu,
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
    init{
        menu.add(this)
    }
}

class MenuItemBooleanDelegate(menu: DelegatedTelemetryMenu,
                              description: String,
                              startingValue: Boolean)
    : MenuItemDelegate<Boolean>(menu, description, startingValue) {

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
    init{
        menu.add(this)
    }
}
class MenuItemEnumDelegate<T>(menu: DelegatedTelemetryMenu,
                              description: String,
                              private vararg val values: T)
    : MenuItemDelegate<T>(menu, description, values[0]) {
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