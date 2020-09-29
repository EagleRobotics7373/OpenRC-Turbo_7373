package org.firstinspires.ftc.teamcode.library.functions

import android.media.MediaPlayer
import android.media.PlaybackParams
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.R

class ExtDirMusicPlayer(extMusicFile: ExtMusicFile, seekAtBeginning: Boolean = false) {

    constructor(extMusicFile: ExtMusicFile):this(extMusicFile, false)

    private val mediaPlayer: MediaPlayer = MediaPlayer()
    var fileIsCorrect = false
        private set
    init {
        mediaPlayer.isLooping = true
        try {
            mediaPlayer.setDataSource("/sdcard/FIRST/music/"+extMusicFile.fileName)
            mediaPlayer.prepare()
            mediaPlayer.seekTo(if (seekAtBeginning) extMusicFile.beginSeekTo else 0)
            fileIsCorrect = true
        } catch (e: Exception) {
            fileIsCorrect = false
        }
    }

    fun play(): Boolean {
        try {
            if (!mediaPlayer.isPlaying) {
                mediaPlayer.start()
            }
        } catch (e : Exception) {
            return false
        }
        return true
    }

    fun pause() {
        mediaPlayer.pause()
    }

    fun stop() {
        mediaPlayer.pause()
        mediaPlayer.stop()
        mediaPlayer.release()
    }

    fun isPlaying() = mediaPlayer.isPlaying
}

enum class ExtMusicFile
constructor (@JvmField val fileName: String, @JvmField val beginSeekTo: Int = 0) {
    UNITY("unity.mp3"),
    MEGALOUNITY("megalounity.mp3"),
    CRABRAVE("crabrave.mp3", 59500),
    BRADTHECHEMIST("bradthechemist.mp3"),
    PACMAN("pacman.mp3"),
    PIZZATIME("pizzatime.mp3"),
    MEGALOVANIA("megalovania.mp3", 16000),
    TETRIS("tetris.mp3"),
    ANGRYBIRDS("angrybirds.mp3"),
    BADPIGGIES("badpiggies.mp3"),
    IMPERIALMARCH("imperialmarch.mp3"),
    MARCHOFTHERESISTANCE("marchoftheresistance.mp3", 15750),
    CANTINABAND("cantinaband.mp3"),
    CREEPER_AWMAN("creeperawman.mp3",38000),
    RIDEOFTHEVALKRIES("rideofthevalkries.mp3"),
    WATERBUFFALO("waterbuffalo.mp3"),
    GOURMETRACE("gourmetrace.mp3"),
    NONE("none")
}