package org.firstinspires.ftc.teamcode.robot.subsystems.barcode

import org.opencv.core.Point
import org.opencv.core.Rect

/**
 * Object to store detection box locations.
 */
object ColorConeConstants {
    /**
     * [DetectionBox] for the detection to occur in.
     */
    @JvmField var boxRight = DetectionBox(
        30,
        130,
        80,
        60,
        150
    )

    /**
     * [DetectionBox] for the detection to occur in.
     */
    @JvmField var boxLeft = DetectionBox(
        160,
        125,
        60,
        60,
        150
    )
}

/**
 * 2d area for EOCV to use.
 *
 * @property x Top left corner x coordinate from the right.
 * @property y Top left corner y coordinate from the top.
 * @property width Box width.
 * @property height Box height.
 * @property threshold Minimum value to count as a detection.
 */
class DetectionBox(
    @JvmField var x: Int,
    @JvmField var y: Int,
    @JvmField var width: Int,
    @JvmField var height: Int,
    @JvmField var threshold: Int
) {
    /**
     * Top left point of the box
     */
    val pointA: Point
        get() = Point(x.toDouble(), y.toDouble())

    /**
     * Bottom right point of the box.
     */
    val pointB: Point
        get() = Point((x + width).toDouble(), (y + height).toDouble())

    /**
     * Rectangle representation of the box.
     */
    val rectangle: Rect
        get() = Rect(pointA, pointB)
}