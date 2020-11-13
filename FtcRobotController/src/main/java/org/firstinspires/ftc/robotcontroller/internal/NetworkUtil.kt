package org.firstinspires.ftc.robotcontroller.internal

import com.qualcomm.robotcore.util.RobotLog
import com.qualcomm.robotcore.wifi.RobotControllerAccessPointAssistant
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import java.io.File
import java.util.*

object NetworkUtil {

    private val file = File("/sdcard/FIRST/network_defaults.txt")
    private val writer = file.writer()
    private val properties = Properties()

    fun loadNetworkSettingsFromFile(): Boolean {
        return if (file.exists()) {
            properties.load(file.reader())
            if (properties.getProperty("name") != null && properties.getProperty("password") != null) {
                    setNetworkSettingsFromProperties()
                    true
                } else {
                    reassignNetworkSettingsFromAccessPointAssistant()
                    false
                }
        } else false

    }

    private fun writeNetworkSettingsToFile() {

        file.delete()
        file.createNewFile()

        properties.store(writer, "Properties Update")
    }

    fun setNetworkSettings(name: String, password: String) {
        setNetworkSettingsWithoutUpdate(name, password)

        setNetworkSettingsFromProperties()
        writeNetworkSettingsToFile()
    }

    private fun setNetworkSettingsWithoutUpdate(name: String, password: String) {
        println("\n\nSetting Wi-Fi properties without update to $name $password\n\n")
        properties.setProperty("name", name)
        properties.setProperty("password", password)
    }

    private fun setNetworkSettingsFromProperties() {
        RobotControllerAccessPointAssistant
                .getRobotControllerAccessPointAssistant(AppUtil.getDefContext())
                .setNetworkSettings(
                        properties.getProperty("name"),
                        properties.getProperty("password"),
                        null
                )
    }

    private fun reassignNetworkSettingsFromAccessPointAssistant() {
        RobotControllerAccessPointAssistant
                .getRobotControllerAccessPointAssistant(AppUtil.getDefContext())
                .let {
                    setNetworkSettingsWithoutUpdate(it.deviceName, it.passphrase)
                }
    }
}