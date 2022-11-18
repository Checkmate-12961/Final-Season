package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.robot.HardwareNames.Servos
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap

/**
 * It's a claw. It grasps... things.
 *
 * @param hardwareMap Map of the hardware passed in from
 * [org.firstinspires.ftc.teamcode.robot.CheckmateRobot].
 */
@Config
class ClumsyClaw(hardwareMap: HardwareMap) : AbstractSubsystem {
    override val tag = "ClumsyClaw"
    override val subsystems = SubsystemMap { tag }

    /**
     * I should make a sex toy out of this.
     */
    private val clawServo = Servos.CLUMSY_CLAW.get(hardwareMap)

    /**
     * Grasp it firmly.
     */
    var grasping = false
        set(value) {
            field = value

            if (value) {
                clawServo.position = closedPosition
            } else {
                clawServo.position = openPosition
            }
            // ...stalck
        }

    init {
        if (Servos.CLUMSY_CLAW.reversed) {
            clawServo.direction = Servo.Direction.REVERSE
        }

        grasping = false
    }

    companion object {
        @JvmField var openPosition = 1.0
        @JvmField var closedPosition = 0.0
    }
}