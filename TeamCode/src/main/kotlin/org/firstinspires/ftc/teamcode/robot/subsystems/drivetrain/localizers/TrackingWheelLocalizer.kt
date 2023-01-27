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

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.robot.HardwareNames
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap
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
class TrackingWheelLocalizer(hardwareMap: HardwareMap) : ThreeTrackingWheelLocalizer(
    /*
     * Our robot this season has the dead wheels rotated 90 degrees from the normal setup. The
     * changes made here are to reflect that rotation.
     */
    listOf(
        Pose2d(LATERAL_DISTANCE / 2.0, 0.0, kotlin.math.PI / 2),  // front
        Pose2d(-LATERAL_DISTANCE / 2.0, 0.0, kotlin.math.PI / 2),  // rear
        Pose2d(0.0, Y_OFFSET, 0.0) // center
    )
), AbstractSubsystem {
    override val tag = this.javaClass.simpleName
    override val subsystems = SubsystemMap { tag }

    private val frontEncoder = HardwareNames.Encoders.FRONT.get(hardwareMap)
    private val rearEncoder = HardwareNames.Encoders.REAR.get(hardwareMap)
    private val centerEncoder = HardwareNames.Encoders.CENTER.get(hardwareMap)

    private val multipliers = listOf(Y_MULTIPLIER, Y_MULTIPLIER, X_MULTIPLIER)

    private val resetValues = onEachEncoder {
        encoderTicksToInches(it.currentPosition.toDouble())
    }

    override fun getWheelPositions(): List<Double> =
        mulList(onEachEncoder { it.currentPosition.toDouble() }, multipliers)

    override fun getWheelVelocities(): List<Double> =
        mulList(onEachEncoder(Encoder::getCorrectedVelocity), multipliers)

    /**
     * Do something on each of the encoders and return the result
     *
     * @param T The return type of the thing you're doing on each encoder
     * @param callback The thing to do on each encoder
     * @return A [List] of elements of type [T] containing the results
     */
    private fun <T> onEachEncoder(callback: (Encoder) -> T): List<T> =
        listOf(frontEncoder, rearEncoder, centerEncoder).map(callback)

    /**
     * Multiply the elements of two [List]s of [Double]
     *
     * @param a The first [List]
     * @param b The second [List]
     * @return The product of [a] and [b]
     */
    private fun mulList(a: List<Double>, b: List<Double>) = a.zip(b) { x: Double, y: Double ->
        x * y
    }

    override fun generateTelemetry(telemetry: Telemetry) {
        val wheelPositions = getWheelPositions()
        telemetry.addData("front", wheelPositions[0] - resetValues[0])
        telemetry.addData("rear", wheelPositions[1] - resetValues[1])
        telemetry.addData("center", wheelPositions[2] - resetValues[2])
    }

    companion object {
        @JvmField var TICKS_PER_REV = 8192.0 // we use REV through bore encoders
        @JvmField var WHEEL_RADIUS = 1.96 / 2 // inches. this is from the andymark listing
        @JvmField var GEAR_RATIO = 1.0 // output (wheel) speed / input (encoder) speed
        @JvmField var LATERAL_DISTANCE = 10.11 // in; distance between the left and right wheels
        @JvmField var Y_OFFSET = -3.55 // in; offset of the lateral wheel
        @JvmField var X_MULTIPLIER = 1.0023 // Multiplier in the X direction
        @JvmField var Y_MULTIPLIER = 1.0047 // Multiplier in the Y direction

        fun encoderTicksToInches(ticks: Double): Double {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
        }
    }
}