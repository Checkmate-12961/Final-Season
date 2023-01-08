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
        get() = Range.scale(liftMotor.targetPosition / ticksPerRevolution, lowerBound, upperBound, 0.0, 1.0)
        set(value) {
            liftMotor.targetPosition = kotlin.math.max(
                Range.scale(
                    kotlin.math.min(value, 1.0),
                    0.0, 1.0,
                    lowerBound, upperBound
                ) * ticksPerRevolution,
                0.0
            ).roundToInt()
        }

    val currentPosition: Double get() = liftMotor.currentPosition / ticksPerRevolution

    override fun generateTelemetry(telemetry: Telemetry) {
        telemetry.addData("targetPosition", targetPosition)
        telemetry.addData("currentPosition", currentPosition)
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
        targetPosition = 0.0
        liftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        liftMotor.power = maxPower
    }

    companion object {
        @JvmField var upperBound = 0.7
        @JvmField var lowerBound = 0.0

        @JvmField var maxPower = 0.9

        @JvmField var ticksPerRevolution = 1425.1
    }
}