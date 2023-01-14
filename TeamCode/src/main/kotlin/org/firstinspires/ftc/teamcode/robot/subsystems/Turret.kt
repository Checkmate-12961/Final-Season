package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.robot.HardwareNames
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap

@Config
class Turret(hardwareMap: HardwareMap) : AbstractSubsystem {
    override val tag = this.javaClass.simpleName
    override val subsystems = SubsystemMap { tag }

    private val turretServo = HardwareNames.Servos.TURRET.get(hardwareMap)

    var locked: Boolean = true
        set(value) {
            field = value

            if (value) {
                position = center
            }
        }

    var position: Double
        get() = turretServo.position
        set(value) {
            turretServo.position = if (locked) center else value
        }

    override fun generateTelemetry(telemetry: Telemetry) {
        telemetry.addData("position", "%.2f", position)
        telemetry.addData("locked", locked)
    }

    companion object {
        @JvmField var center = 0.55
    }
}