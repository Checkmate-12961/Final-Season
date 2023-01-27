package org.firstinspires.ftc.teamcode.opmodes.powerplay.tele

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.abstracts.SuperController
import org.firstinspires.ftc.teamcode.robot.abstracts.Triggerables

@TeleOp(group = "TEST")
class TestControllers : BaseOpMode() {
    override fun preRunLoop() {
        val controllers = listOf(gp1, gp2)
        for (i in controllers.indices) {
            val controller = controllers[i]
            val id = i + 1

            mapOf<String, (SuperController) -> Triggerables.ControllerButton>(
                // face buttons
                "a" to SuperController::a,
                "b" to SuperController::b,
                "x" to SuperController::x,
                "y" to SuperController::y,

                // dpad buttons
                "dpad up" to SuperController::dpadUp,
                "dpad right" to SuperController::dpadRight,
                "dpad down" to SuperController::dpadDown,
                "dpad left" to SuperController::dpadLeft,

                // depress joysticks
                "left joystick button" to SuperController::leftStickButton,
                "right joystick button" to SuperController::rightStickButton,

                // bumpers
                "left bumper" to SuperController::leftBumper,
                "right bumper" to SuperController::rightBumper,

                // misc
                "options" to SuperController::options,
                "share" to SuperController::share,
                "home" to SuperController::ps
            ).forEach { (name, getButton) ->
                val button = getButton(controller)
                telemetry.addLine("Press $name on game pad $id")
                telemetry.update()

                // wait for the input to come through
                while (!button.rawPressed() && !isStopRequested) continue
            }

            mapOf<String, (SuperController) -> Triggerables.ControllerStick>(
                // left
                "left stick x" to SuperController::leftStickX,
                "left stick y" to SuperController::leftStickY,

                // right
                "right stick x" to SuperController::rightStickX,
                "right stick y" to SuperController::rightStickY
            ).forEach { (name, getStick) ->
                val stick = getStick(controller)
                listOf(-1, 1).forEach { coefficient ->
                    while (stick.rawValue() * coefficient <= .95 && !isStopRequested) {
                        telemetry.addLine("Bring $name on game pad $id to $coefficient")
                        telemetry.addLine("stick value ${stick.rawValue()}")
                        telemetry.update()
                    }
                }
            }

            mapOf<String, (SuperController) -> Triggerables.ControllerTrigger>(
                "left trigger" to SuperController::leftTrigger,
                "right trigger" to SuperController::rightTrigger
            ).forEach { (name, getTrigger) ->
                val trigger = getTrigger(controller)

                while (trigger.rawValue() <= .95 && !isStopRequested) {
                    telemetry.addLine("Fully depress $name on game pad $id")
                    telemetry.addLine("stick value: ${trigger.rawValue()}")
                    telemetry.update()
                }
            }
        }

        telemetry.addLine("Success!")
        telemetry.update()
        sleep(2000)

        requestOpModeStop()
    }
}