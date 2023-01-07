package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.robot.HardwareNames.Motors
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap
import kotlin.math.abs
import kotlin.math.sign

@Config
class LiftyLinkage(hardwareMap: HardwareMap) : AbstractSubsystem {
    override val tag = this.javaClass.simpleName
    override val subsystems = SubsystemMap { tag }

    private val liftMotorA = Motors.LIFTY_LINKAGE_A.get(hardwareMap)
    private val liftMotorB = Motors.LIFTY_LINKAGE_B.get(hardwareMap)

    private val motors = listOf(liftMotorA, liftMotorB)

    enum class Action(val power: () -> Double) {
        DOWN({ downPower }),
        UP({ upPower }),
        REST({ restPower }),
        HOLD({ holdPower})
    }

    var speed = 0.0

    var currentSpeed = 0.0
        private set

    fun action(action: Action) {
        speed = action.power()
    }

    override fun loop() {
        // makes the robot not shit itself as badly

        currentSpeed = liftMotorA.power

        val delta = sign(speed - currentSpeed) * adjustmentRate
        val newPower = if (
                abs(abs(currentSpeed + delta) - abs(speed)) > adjustmentRate
            ) currentSpeed + delta
            else speed

        if (delta != 0.0) {
            motors.forEach { it.power = newPower }
        }
    }

    override fun generateTelemetry(telemetry: Telemetry) {
        telemetry.addData("liftTargetVel", speed)
        telemetry.addData("liftRealVel", currentSpeed)
        telemetry.addData("liftVelDelta", currentSpeed - speed)
    }

    init {
        // Initialize the motors
        motors.forEach { motor ->
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            motor.power = 0.0
        }
    }

    companion object {
        @JvmField var adjustmentRate = 0.03
        @JvmField var upPower = .6
        @JvmField var restPower = .0
        @JvmField var holdPower = .2
        @JvmField var downPower = -.3
    }
}