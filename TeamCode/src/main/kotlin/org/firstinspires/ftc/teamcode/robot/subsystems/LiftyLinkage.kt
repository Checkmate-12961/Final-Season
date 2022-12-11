package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.robot.HardwareNames.Motors
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap

@Config
class LiftyLinkage(hardwareMap: HardwareMap) : AbstractSubsystem {
    override val tag = "LiftyLinkage"
    override val subsystems = SubsystemMap { tag }

    private val liftMotorA = Motors.LIFTY_LINKAGE_A.get(hardwareMap)
    private val liftMotorB = Motors.LIFTY_LINKAGE_B.get(hardwareMap)

    private val motors = listOf(liftMotorA, liftMotorB)

    enum class Action(val power: () -> Double) {
        DOWN({ downPower }),
        UP({ upPower }),
        HOLD({ holdPower })
    }

    var speed: Double
        get() = liftMotorA.power
        set(value) = motors.forEach { it.power = value }

    fun action(action: Action) {
        speed = action.power()
    }

    init {
        // Initialize the motors
        motors.forEach { motor ->
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            motor.power = 0.0
        }
    }

    companion object {
        @JvmField var upPower = 0.7
        @JvmField var downPower = -0.3
        @JvmField var holdPower = 0.1
    }
}