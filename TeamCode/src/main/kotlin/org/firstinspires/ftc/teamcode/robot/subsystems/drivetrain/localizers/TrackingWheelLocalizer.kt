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
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.robot.HardwareNames
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap

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
class TrackingWheelLocalizer(hardwareMap: HardwareMap) : AbstractSubsystem, ThreeTrackingWheelLocalizer(
    /*
     * Our robot this season has the dead wheels rotated 90 degrees from the normal setup. The
     * changes made here are to reflect that rotation.
     */
    listOf(
        Pose2d(LATERAL_DISTANCE / 2.0, 0.0, kotlin.math.PI / 2),  // front
        Pose2d(-LATERAL_DISTANCE / 2.0, 0.0, kotlin.math.PI / 2),  // rear
        Pose2d(0.0, Y_OFFSET, 0.0) // center
    )
) {
    override val tag = this.javaClass.simpleName
    override val subsystems = SubsystemMap { tag }

    private val frontEncoder = HardwareNames.Encoders.FRONT.get(hardwareMap)
    private val rearEncoder = HardwareNames.Encoders.REAR.get(hardwareMap)
    private val centerEncoder = HardwareNames.Encoders.CENTER.get(hardwareMap)

    override fun getWheelPositions(): List<Double> {
        return listOf(
            encoderTicksToInches(frontEncoder.currentPosition.toDouble()) * Y_MULTIPLIER,
            encoderTicksToInches(rearEncoder.currentPosition.toDouble()) * Y_MULTIPLIER,
            encoderTicksToInches(centerEncoder.currentPosition.toDouble()) * X_MULTIPLIER
        )
    }

    override fun getWheelVelocities(): List<Double> {
        // DONE: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method
        return listOf(
            encoderTicksToInches(frontEncoder.correctedVelocity * Y_MULTIPLIER),
            encoderTicksToInches(rearEncoder.correctedVelocity * Y_MULTIPLIER),
            encoderTicksToInches(centerEncoder.correctedVelocity * X_MULTIPLIER)
        )
    }

    override fun generateTelemetry(telemetry: Telemetry) {
        val (front, rear, center) = getWheelPositions()

        telemetry.addData("front", "%.2f", front)
        telemetry.addData("rear", "%.2f", rear)
        telemetry.addData("center", "%.2f", center)
    }

    companion object {
        @JvmField var TICKS_PER_REV = 8192.0 // we use REV through bore encoders
        @JvmField var WHEEL_RADIUS = 1.96 / 2 // inches. this is from the andymark listing
        @JvmField var GEAR_RATIO = 1.0 // output (wheel) speed / input (encoder) speed
        @JvmField var LATERAL_DISTANCE = 10.69 // in; distance between the left and right wheels
        @JvmField var Y_OFFSET = -3.55 // in; offset of the lateral wheel
        @JvmField var X_MULTIPLIER = 1.0023 // Multiplier in the X direction
        @JvmField var Y_MULTIPLIER = 1.0047 // Multiplier in the Y direction

        fun encoderTicksToInches(ticks: Double): Double {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
        }
    }
}