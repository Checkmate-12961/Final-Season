package org.firstinspires.ftc.teamcode.robot.subsystems.colorcone

import com.acmerobotics.dashboard.config.Config
import org.opencv.core.Point
import org.opencv.core.Rect

/**
 * Object to store detection box locations.
 */
@Config
object ColorConeConstants {
    /**
     * [DetectionBox] for the detection to occur in.
     */

    @JvmField var boxRight = DetectionBox(
        30,
        90,
        50,
        75
    )

    /**
     * [DetectionBox] for the detection to occur in.
     */
    @JvmField var boxLeft = DetectionBox(
        220,
        75,
        50,
        70
    )
}

/**
 * 2d area for EOCV to use.
 *
 * @property x Top left corner x coordinate from the right.
 * @property y Top left corner y coordinate from the top.
 * @property width Box width.
 * @property height Box height.
 */
data class DetectionBox(
    @JvmField var x: Int,
    @JvmField var y: Int,
    @JvmField var width: Int,
    @JvmField var height: Int
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