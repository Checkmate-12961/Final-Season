package org.firstinspires.ftc.teamcode.robot.subsystems.barcode

import com.acmerobotics.dashboard.config.Config
import org.opencv.core.Point
import org.opencv.core.Rect

/**
 * Object to store detection box locations.
 */
@Config
object BarcodeConstants {
    /**
     * [DetectionBox] for the left side.
     */
    @JvmField var leftBox = DetectionBox(
        80,
        130,
        50,
        50,
        150
    )

    /**
     * [DetectionBox] for the middle.
     */
    @JvmField var middleBox = DetectionBox(
        255,
        135,
        50,
        50,
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