package org.firstinspires.ftc.teamcode.library.functions.telemetrymenu

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.library.functions.Position
import kotlin.reflect.KMutableProperty
import kotlin.reflect.KProperty

class IterableTelemetryMenu constructor(val telemetry: Telemetry, vararg items: MenuItem) {
    private val list = items.toList()
    var current = list[0]
    fun refresh() {
        list.forEach {
            telemetry.addData(if (it === current) "-> " else "" + it.description,
                    if (it.canIterateBackward()) " << " else ""
                            + it
                            + if (it.canIterateForward()) " >> " else "")
        }
        telemetry.update()

    }

    fun nextItem() {
        val currentNum = list.indexOf(current)
        if (currentNum < list.size-1) current = list[currentNum+1]
        refresh()
    }
    fun previousItem() {
        val currentNum = list.indexOf(current)
        if (currentNum > 0) current = list[currentNum-1]
        refresh()
    }
    fun iterateCurrentItemForward() {
        if (current.canIterateForward())
            current.iterateForward()
        refresh()
    }
    fun iterateCurrentItemBackward() {
        if (current.canIterateBackward())
            current.iterateBackward()
        refresh()
    }
}

abstract class MenuItem
constructor(val description: String) {
    abstract override fun toString(): String
    abstract fun iterateForward()
    abstract fun iterateBackward(): Unit
    abstract fun canIterateForward(): Boolean
    abstract fun canIterateBackward(): Boolean
}

class MenuItemInteger
constructor(description: String,
            private val property: KMutableProperty<Int>,
            private val lowerLimit: Int,
            private val upperLimit: Int,
            private val incrementBy: Int)
    : MenuItem(description) {
    override fun toString(): String = property.toString()
    override fun iterateForward() {
        if (canIterateForward())
            property.setter.call(property.getter.call() + incrementBy)
    }

    override fun iterateBackward() {
        if (canIterateBackward())
            property.setter.call(property.getter.call() - incrementBy)
    }

    override fun canIterateForward(): Boolean =
            property.getter.call() + incrementBy <= upperLimit

    override fun canIterateBackward(): Boolean =
            property.getter.call() - incrementBy >= lowerLimit

}

class MenuItemBoolean
constructor(description: String,
            private val property: KMutableProperty<Boolean>
) : MenuItem(description) {
    override fun toString(): String = property.getter.call().toString()
    override fun iterateForward() {
        if (canIterateForward())
            property.setter.call(true)
    }

    override fun iterateBackward() {
        if (canIterateBackward())
            property.setter.call(false)
    }

    override fun canIterateForward(): Boolean =
            !property.getter.call()

    override fun canIterateBackward(): Boolean =
            property.getter.call()
}

class MenuItemEnum<T>
constructor(description: String,
            private val property: KMutableProperty<T>,
            vararg allowedElements: T)
    : MenuItem(description) {
    val iterator = allowedElements
            .toList()
            .listIterator()

    override fun toString(): String =
            property.getter.call().toString()

    override fun iterateForward() {
        if (canIterateForward()) property.setter.call(iterator.next())
    }

    override fun iterateBackward() {
        if (canIterateBackward()) property.setter.call(iterator.previous())
    }

    override fun canIterateForward(): Boolean =
            iterator.hasNext()


    override fun canIterateBackward(): Boolean =
            iterator.hasPrevious()
}

