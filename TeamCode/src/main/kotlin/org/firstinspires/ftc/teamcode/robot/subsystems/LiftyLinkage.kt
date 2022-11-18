package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.robot.HardwareNames.Motors
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap
import kotlin.math.roundToInt

@Config
class LiftyLinkage(hardwareMap: HardwareMap) : AbstractSubsystem {
    override val tag = "LiftyLinkage"
    override val subsystems = SubsystemMap { tag }

    private val liftMotor = Motors.LIFTY_LINKAGE.get(hardwareMap)

    var height: Double
        get() = liftMotor.targetPosition / ticksPerRev
        set(value) {
            liftMotor.targetPosition = (value * ticksPerRev).roundToInt()
        }

    data class LiftBounds(@JvmField var min: Double, @JvmField var max: Double)

    init {
        // Initialize the motor
        liftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        // Set it to run to a target position and hold it
        liftMotor.targetPosition = 0
        liftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        liftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        liftMotor.power = motorSpeed
    }

    companion object {
        @JvmField var liftBounds = LiftBounds(0.0, 20.0)
        @JvmField var motorSpeed = 1.0

        @JvmField var ticksPerRev = 1.0
    }
}