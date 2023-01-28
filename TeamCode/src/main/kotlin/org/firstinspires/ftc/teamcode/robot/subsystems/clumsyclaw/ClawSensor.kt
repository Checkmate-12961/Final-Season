package org.firstinspires.ftc.teamcode.robot.subsystems.clumsyclaw

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.robot.HardwareNames
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import org.firstinspires.ftc.teamcode.robot.abstracts.SubsystemMap

@Config
class ClawSensor(hardwareMap: HardwareMap) : AbstractSubsystem {
    override val tag = this.javaClass.simpleName
    override val subsystems = SubsystemMap { tag }

    private val sensor = HardwareNames.DistanceSensors.CLAW_SENSOR.get(hardwareMap)

    val distance: Double
        get() = sensor.getDistance(DistanceUnit.INCH)

    val close: Boolean
        get() = distance <= closeDistance

    override fun generateTelemetry(telemetry: Telemetry) {
        telemetry.addData("distance", "%.3f", distance)
    }

    companion object {
        @JvmField var closeDistance = 1.0
    }
}