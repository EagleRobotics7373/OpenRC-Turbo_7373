package org.firstinspires.ftc.teamcode.library.vision.skystone

import org.firstinspires.ftc.robotcore.external.tfod.Recognition

@Throws(NoSuchElementException::class)
fun findBlockOffsetFromRecognitions(recognitions: List<Recognition>):Float {
    recognitions.toMutableList().sortByDescending { it.width }
    val recognition = recognitions.first()
    if (recognitions.isEmpty()) throw NoSuchElementException()
    val recognitionCenter = recognition.left + recognition.right / 2
    val frameCenter = recognition.imageWidth / 2

    return frameCenter - recognitionCenter
}