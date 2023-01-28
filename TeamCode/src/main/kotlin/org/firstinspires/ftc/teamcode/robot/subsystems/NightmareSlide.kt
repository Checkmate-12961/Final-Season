package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.robot.HardwareNames
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap
import kotlin.reflect.KProperty

@Config
class NightmareSlide(hardwareMap: HardwareMap) : AbstractSubsystem {
    override val tag = this.javaClass.simpleName
    override val subsystems = SubsystemMap { tag }

    private val bottomServo = HardwareNames.Servos.SLIDE_A.get(hardwareMap)
    private val topServo = HardwareNames.Servos.SLIDE_B.get(hardwareMap)

    private var bottom: Double by ServoDelegate(
        bottomServo,
        { Math.toRadians(bottomServoIntrinsic) },
        Math.toRadians(270.0),
        Math.toRadians(90.0)
    )
    private var top: Double by ServoDelegate(
        topServo,
        { Math.toRadians(topServoIntrinsic) },
        Math.toRadians(270.0),
        Math.toRadians(90.0)
    )

    data class Keyframe(@JvmField var bottomPos: Double = 0.0, @JvmField var topPos: Double = 0.0) {
        fun toRadians() = Keyframe(Math.toRadians(bottomPos), Math.toRadians(topPos))
    }

    data class Bounds(@JvmField var lowerBound: Double = 0.0, @JvmField var upperBound: Double = 0.0)

    /**
     * Manages applying values from the enum to the servos.
     *
     * @param servo [Servo] to apply the position to.
     * @param default Starting position for the servo.
     */
    private class ServoDelegate(
        private val servo: Servo,
        private val getPointFiveAngle: () -> Double,
        private val rangeOfMotion: Double,
        default: Double
    ) {
        private val upperBound = getPointFiveAngle() + rangeOfMotion / 2
        private val lowerBound = getPointFiveAngle() - rangeOfMotion / 2

        /**
         * Current position of [servo].
         */
        private var field: Double = normalize(default)

        private fun normalize(value: Double): Double {
            var normalized = value

            while (normalized > upperBound) normalized -= 2 * kotlin.math.PI
            while (normalized < lowerBound) normalized += 2 * kotlin.math.PI

            normalized = kotlin.math.min(normalized, upperBound)

            return normalized
        }

        private fun scale(value: Double): Double {
            return (normalize(value) - (getPointFiveAngle() - rangeOfMotion / 2)) / rangeOfMotion
        }

        operator fun getValue(thisRef: Any?, property: KProperty<*>): Double = field
        operator fun setValue(thisRef: Any?, property: KProperty<*>, value: Double) {
            field = value
            servo.position = scale(value)
        }
    }

    var adjustment: Double = 0.0
        set(value) {
            field = Range.clip(value, 0.0, 1.0)
            currentFrame = currentFrame
        }

    var currentFrame: Int = -1
        set(value) {
            field = Range.clip(value, -1, keyframes.size)

            val frame = when (field) {
                -1 -> Keyframe(
                    FRAME_B1.bottomPos,
                    Range.scale(
                        1.0 - adjustment,
                        0.0,
                        1.0,
                        B_LOCK_BOUNDS.lowerBound,
                        B_LOCK_BOUNDS.upperBound
                    )
                )
                keyframes.size -> Keyframe(
                    (FRAME_A3.bottomPos - kotlin.math.sign(adjustment) * FRAME_A3.bottomPos) / 2.0,
                    Range.scale(
                        1.0 - adjustment,
                        0.0,
                        1.0,
                        A_LOCK_BOUNDS.lowerBound,
                        A_LOCK_BOUNDS.upperBound
                    )
                )
                else -> keyframes[field]
            }.toRadians()

            top = frame.topPos
            bottom = frame.bottomPos
        }

    private val keyframes = listOf(
        FRAME_0,
        FRAME_A1,
        FRAME_A2
    )

    override fun generateTelemetry(telemetry: Telemetry) {
        telemetry.addData("bottomAngle", Math.toDegrees(bottom))
        telemetry.addData("topAngle", Math.toDegrees(top))
        telemetry.addData("frame", currentFrame)
        telemetry.addData("adjustment", adjustment)
    }

    init {
        top = FRAME_INIT.topPos
        bottom = FRAME_INIT.bottomPos
    }

    override fun preLoop() {
        currentFrame = 0
    }

    companion object {
        @JvmField var FRAME_B1 = Keyframe(135.0, -1.0)
        @JvmField var FRAME_0 = Keyframe(90.0, 90.0)
        @JvmField var FRAME_A1 = Keyframe(45.0, 135.0)
        @JvmField var FRAME_A2 = Keyframe(45.0, 180.0)
        @JvmField var FRAME_A3 = Keyframe(15.0, -1.0)

        @JvmField var A_LOCK_BOUNDS = Bounds(190.0, 300.0)
        @JvmField var B_LOCK_BOUNDS = Bounds(45.0, 135.0)

        @JvmField var FRAME_INIT = Keyframe(90.0, 90.0)

        @JvmField var bottomServoIntrinsic = 20.0
        @JvmField var topServoIntrinsic = 165.0
    }
}