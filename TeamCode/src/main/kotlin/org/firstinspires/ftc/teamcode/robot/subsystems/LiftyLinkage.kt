package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.robot.HardwareNames.Motors
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap
import kotlin.math.roundToInt

@Config
class LiftyLinkage(hardwareMap: HardwareMap) : AbstractSubsystem {
    override val tag = this.javaClass.simpleName
    override val subsystems = SubsystemMap { tag }

    private val liftMotor = Motors.LIFTY_LINKAGE.get(hardwareMap)

    var targetPosition: Double
        get() = Range.scale(
            liftMotor.targetPosition / ticksPerRevolution,
            lowerBound,
            upperBound,
            0.0,1.0
        )
        set(value) {
            liftMotor.targetPosition = kotlin.math.max(
                Range.scale(
                    kotlin.math.min(
                        if (value < middleBound && lockedAboveMid) middleBound else value,
                        1.0
                    ),
                    0.0, 1.0,
                    lowerBound, upperBound
                ) * ticksPerRevolution,
                0.0
            ).roundToInt()
        }

    val isAboveMid: Boolean get() = currentPosition >= middleBound

    var lockedAboveMid = false
        set(value) {
            field = value

            if (value) {
                targetPosition = kotlin.math.max(targetPosition, middleBound)
            }
        }

    val currentPosition: Double get() = Range.scale(
        liftMotor.currentPosition / ticksPerRevolution,
        lowerBound,
        upperBound,
        0.0,
        1.0
    )

    override fun generateTelemetry(telemetry: Telemetry) {
        telemetry.addData("targetPosition", targetPosition)
        telemetry.addData("currentPosition", currentPosition)
        telemetry.addData("currentRawPosition", liftMotor.currentPosition / ticksPerRevolution)
        liftMotor.getCurrent(CurrentUnit.AMPS).let { current ->
            telemetry.addData(
                "current",
                "%.2fA (%.2f%%)",
                current, current * 10.0
            )
        }
    }

    init {
        // Initialize the motor
        liftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        if (!tuningMode) {
            targetPosition = 0.0
            liftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
            liftMotor.power = maxPower
        } else {
            liftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
    }

    companion object {
        @JvmField var upperBound = 0.7
        @JvmField var middleBound = 0.32
        @JvmField var lowerBound = 0.0

        @JvmField var tuningMode = false

        @JvmField var maxPower = 0.9

        @JvmField var ticksPerRevolution = 1425.1
    }
}