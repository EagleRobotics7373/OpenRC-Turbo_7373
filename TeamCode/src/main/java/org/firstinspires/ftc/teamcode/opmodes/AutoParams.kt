package org.firstinspires.ftc.teamcode.opmodes

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.library.functions.ExtMusicFile
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.DelegatedTelemetryMenu
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.MenuItemEnumDelegate
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.MenuItemIntDelegate

class AutoParamMenu(telemetry: Telemetry) {

    @JvmField val menu = DelegatedTelemetryMenu(telemetry)
    var musicFile : ExtMusicFile by MenuItemEnumDelegate(menu, "Music",ExtMusicFile.NONE, ExtMusicFile.UNITY, ExtMusicFile.MEGALOUNITY)
    var angle : Int by MenuItemIntDelegate(menu, "Angle", 0, 0, 180, 30)

}