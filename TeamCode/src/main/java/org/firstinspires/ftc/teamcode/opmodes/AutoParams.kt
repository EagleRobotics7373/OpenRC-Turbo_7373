package org.firstinspires.ftc.teamcode.opmodes

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.library.functions.ExtDirMusicPlayer
import org.firstinspires.ftc.teamcode.library.functions.ExtMusicFile
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.*

class AutoMenuControllerIterative(telemetry: Telemetry) {
    @JvmField var musicFile : ExtMusicFile = ExtMusicFile.NONE
    @JvmField var driveTime : Int = 1000
    @JvmField val menu = IterableTelemetryMenu(telemetry,
            MenuItemEnum("Music",::musicFile, ExtMusicFile.NONE, ExtMusicFile.UNITY, ExtMusicFile.MEGALOUNITY, ExtMusicFile.BRADTHECHEMIST),
            MenuItemInteger("Driving Time", ::driveTime, 0, 5000, 500))

}

class AutoMenuControllerDelegated(telemetry: Telemetry) {
    @JvmField val menu = DelegatedTelemetryMenu(telemetry)
    var musicFile : ExtMusicFile by MenuItemEnumDelegate(menu, "Music",ExtMusicFile.NONE, ExtMusicFile.UNITY, ExtMusicFile.MEGALOUNITY)
//    var angle : Int by MenuItemIntDelegate(menu, "Angle", 0, 0, 180, 30)
    var driveTime : Int by MenuItemIntDelegate(menu, "time", 0,0, 5000, 500)

}