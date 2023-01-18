package org.firstinspires.ftc.teamcode.opmodes.powerplay.auto.util

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import kotlin.math.cos
import kotlin.math.sin

class OpModeUtil {
    class SuperPose2d(
        @JvmField var x: Double = 0.0,
        @JvmField var y: Double = 0.0,
        @JvmField var h: Double = 0.0
    ) {
        var pose2d: Pose2d
            get() = Pose2d(x, y, hRad)
            set(value) {
                x = value.x
                y = value.y
                hRad = value.heading
            }

        var hRad: Double
            get() = h * kotlin.math.PI / 180.0
            set(value) {
                h = value * 180.0 / kotlin.math.PI
            }

        var vector2d: Vector2d
            get() = Vector2d(x, y)
            set(value) {
                x = value.x
                y = value.y
                h = 0.0
            }

        /**
         * Transforms a Pose2d by another Pose2d.
         * [A simulator for this behavior can be found here.](https://www.desmos.com/calculator/0x7xoeiudh)
         *
         * @param b The pose to transform the first pose by.
         * @return The result of the transformation.
         */
        fun transformBy(b: Pose2d) {
            val cosT = cos(hRad)
            val sinT = sin(hRad)
            x += b.x * cosT - b.y * sinT
            y += b.x * sinT + b.y * cosT
            hRad += b.heading
        }

        companion object {
            @JvmStatic fun fromPose2d(pose: Pose2d): SuperPose2d {
                val newPose = SuperPose2d(pose.x, pose.y)
                newPose.hRad = pose.heading
                return newPose
            }
        }
    }
}

fun Pose2d.toSuperPose2d(): OpModeUtil.SuperPose2d {
    return OpModeUtil.SuperPose2d.fromPose2d(this)
}