package org.firstinspires.ftc.teamcode.robot.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.robot.HardwareNames
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap
import org.firstinspires.ftc.teamcode.robot.util.DumbassProgrammerError
import kotlin.reflect.KProperty

class NightmareSlide(hardwareMap: HardwareMap) : AbstractSubsystem {
    override val tag = this.javaClass.simpleName
    override val subsystems = SubsystemMap { tag }

    private val bottomServo = HardwareNames.Servos.SLIDE_A.get(hardwareMap)
    private val topServo = HardwareNames.Servos.SLIDE_B.get(hardwareMap)

    private var bottom: Double by ServoDelegate(
        bottomServo,
        Math.toRadians(60.0),
        Math.toRadians(270.0),
        Math.toRadians(90.0)
    )
    private var top: Double by ServoDelegate(
        topServo,
        Math.toRadians(185.0),
        Math.toRadians(270.0),
        Math.toRadians(90.0)
    )

    data class Keyframe(@JvmField var bottomPos: Double = 0.0, @JvmField var topPos: Double = 0.0) {
        companion object {
            fun fromDegrees(bottomPos: Double, topPos: Double) =
                Keyframe(Math.toRadians(bottomPos), Math.toRadians(topPos))
        }
    }

    /**
     * Manages applying values from the enum to the servos.
     *
     * @param servo [Servo] to apply the position to.
     * @param default Starting position for the servo.
     */
    private class ServoDelegate(
        private val servo: Servo,
        private val pointFiveAngle: Double,
        private val rangeOfMotion: Double,
        default: Double
    ) {
        private val upperBound = pointFiveAngle + rangeOfMotion / 2
        private val lowerBound = pointFiveAngle - rangeOfMotion / 2

        /**
         * Current position of [servo].
         */
        private var field: Double = normalize(default)

        private fun normalize(value: Double): Double {
            var normalized = value

            while (normalized > upperBound) normalized -= 2 * kotlin.math.PI
            while (normalized < lowerBound) normalized += 2 * kotlin.math.PI

            if (normalized > upperBound) throw DumbassProgrammerError("Angle out of servo bounds. upper: $upperBound, lower: $lowerBound, value: $normalized")

            return normalized
        }

        private fun scale(value: Double): Double {
            return (normalize(value) - (pointFiveAngle - rangeOfMotion / 2)) / rangeOfMotion
        }

        operator fun getValue(thisRef: Any?, property: KProperty<*>): Double = field
        operator fun setValue(thisRef: Any?, property: KProperty<*>, value: Double) {
            field = value
            servo.position = scale(value)
        }
    }

    var currentFrame: Int = 0
        set(value) {
            field = value

            val frame = keyframes[value]

            top = frame.topPos
            bottom = frame.bottomPos
        }

    private val keyframes = listOf(
        FRAME_B1,
        FRAME_0,
        FRAME_A1,
        FRAME_A2,
        FRAME_A3
    )

    companion object {
        @JvmField var FRAME_B1 = Keyframe.fromDegrees(135.0, 90.0)
        @JvmField var FRAME_0 = Keyframe.fromDegrees(90.0, 90.0)
        @JvmField var FRAME_A1 = Keyframe.fromDegrees(45.0, 135.0)
        @JvmField var FRAME_A2 = Keyframe.fromDegrees(45.0, 180.0)
        @JvmField var FRAME_A3 = Keyframe.fromDegrees(0.0, 270.0)
    }

    override fun generateTelemetry(telemetry: Telemetry) {
        telemetry.addData("bottomAngle", Math.toDegrees(bottom))
        telemetry.addData("topAngle", Math.toDegrees(top))
        telemetry.addData("frame", currentFrame)
    }

    init {
        currentFrame = 1
    }
}