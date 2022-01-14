package org.firstinspires.ftc.teamcode.robot.subsystems.barcode

import com.acmerobotics.dashboard.config.Config
import org.opencv.core.Point
import org.opencv.core.Rect

@Config
object BarcodeConstants {
    @JvmField var leftBox = DetectionBox(
        80,
        130,
        50,
        50,
        150
    )
    @JvmField var middleBox = DetectionBox(
        255,
        135,
        50,
        50,
        150
    )
}

class DetectionBox(
    @JvmField var x: Int,
    @JvmField var y: Int,
    @JvmField var width: Int,
    @JvmField var height: Int,
    @JvmField var threshold: Int
) {
    val pointA: Point
        get() = Point(x.toDouble(), y.toDouble())

    val pointB: Point
        get() = Point((x + width).toDouble(), (y + height).toDouble())

    val rectangle: Rect
        get() = Rect(pointA, pointB)
}