package org.firstinspires.ftc.teamcode.opmodes.powerplay.auto

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.robot.HardwareNames

@Config
@Autonomous(group = "TEST")
class TestDriveMotors : LinearOpMode() {
    override fun runOpMode() {
        val motors = mutableMapOf<HardwareNames.Motors, DcMotorEx?>(
            HardwareNames.Motors.LEFT_FRONT to null,
            HardwareNames.Motors.LEFT_REAR to null,
            HardwareNames.Motors.RIGHT_FRONT to null,
            HardwareNames.Motors.RIGHT_REAR to null
        )

        motors.forEach {
            val motor = it.key.get(hardwareMap)
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            motor.power = 0.0
            motors[it.key] = motor

        }

        waitForStart()

        motors.forEach {
            telemetry.addLine("Testing ${it.key.name} forward...")
            telemetry.update()
            it.value?.power = power
            sleep(delay)
            telemetry.addLine("Testing ${it.key.name} backwards...")
            telemetry.update()
            it.value?.power = -power
            sleep(delay)
            it.value?.power = 0.0
        }
    }

    companion object {
        @JvmField var power = .3
        @JvmField var delay = 3000L
    }
}