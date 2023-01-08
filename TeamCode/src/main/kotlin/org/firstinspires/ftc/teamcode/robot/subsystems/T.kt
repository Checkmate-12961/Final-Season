package org.firstinspires.ftc.teamcode.robot.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.robot.HardwareNames
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap

class T(hardwareMap: HardwareMap) : AbstractSubsystem {
    override val tag = this.javaClass.simpleName
    override val subsystems = SubsystemMap { tag }

    private val turretServo = HardwareNames.Servos.TURRET.get(hardwareMap)

    var position: Double
        get() = turretServo.position
        set(value) {
            turretServo.position = value
        }
}