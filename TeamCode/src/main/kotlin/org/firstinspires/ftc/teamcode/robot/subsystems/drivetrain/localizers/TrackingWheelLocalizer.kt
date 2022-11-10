/*
The MIT License (MIT)

Copyright © 2021 Checkmate Robotics

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the “Software”), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.localizers

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.robot.HardwareNames
import org.firstinspires.ftc.teamcode.robot.util.Encoder

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
class TrackingWheelLocalizer(hardwareMap: HardwareMap) : ThreeTrackingWheelLocalizer(
    listOf(
        Pose2d(0.0, LATERAL_DISTANCE / 2.0, 0.0),  // left
        Pose2d(0.0, -LATERAL_DISTANCE / 2.0, 0.0),  // right
        Pose2d(FORWARD_OFFSET, 0.0, Math.toRadians(90.0)) // front
    )
) {
    private val leftEncoder: Encoder
    private val rightEncoder: Encoder
    private val frontEncoder: Encoder
    override fun getWheelPositions(): List<Double> {
        return listOf(
            encoderTicksToInches(leftEncoder.currentPosition.toDouble()) * X_MULTIPLIER,
            encoderTicksToInches(rightEncoder.currentPosition.toDouble()) * X_MULTIPLIER,
            encoderTicksToInches(frontEncoder.currentPosition.toDouble()) * Y_MULTIPLIER
        )
    }

    override fun getWheelVelocities(): List<Double> {
        // DONE: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method
        return listOf(
            encoderTicksToInches(leftEncoder.correctedVelocity),
            encoderTicksToInches(rightEncoder.correctedVelocity),
            encoderTicksToInches(frontEncoder.correctedVelocity)
        )
    }

    companion object {
        @JvmField var TICKS_PER_REV = 8192.0
        @JvmField var WHEEL_RADIUS = 1.0 // in
        @JvmField var GEAR_RATIO = 1.0 // output (wheel) speed / input (encoder) speed
        @JvmField
        var LATERAL_DISTANCE = 8.12153 // in; distance between the left and right wheels
        @JvmField var FORWARD_OFFSET = -4.0 // in; offset of the lateral wheel
        @JvmField var X_MULTIPLIER = 1.0 // Multiplier in the X direction
        @JvmField var Y_MULTIPLIER = 1.0 // Multiplier in the Y direction

        fun encoderTicksToInches(ticks: Double): Double {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
        }
    }

    init {
        rightEncoder = Encoder(
            hardwareMap.get(DcMotorEx::class.java, HardwareNames.Encoders.REAR.id)
        )
        leftEncoder = Encoder(
            hardwareMap.get(DcMotorEx::class.java, HardwareNames.Encoders.FRONT.id)
        )
        frontEncoder = Encoder(
            hardwareMap.get(DcMotorEx::class.java, HardwareNames.Encoders.CENTER.id)
        )

        if (HardwareNames.Encoders.REAR.reverse) {
            rightEncoder.direction = Encoder.Direction.REVERSE
        }
        if (HardwareNames.Encoders.FRONT.reverse) {
            leftEncoder.direction = Encoder.Direction.REVERSE
        }
        if (HardwareNames.Encoders.CENTER.reverse) {
            frontEncoder.direction = Encoder.Direction.REVERSE
        }
    }
}