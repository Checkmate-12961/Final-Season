/*
The MIT License (MIT)

Copyright © 2021 Checkmate Robotics

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the “Software”), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.opmodes.powerplay.tele

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.robot.TheLegend
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.ClumsyClaw
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret
import org.firstinspires.ftc.teamcode.robot.subsystems.camera.ConePositionPipeline

@Config
@TeleOp(name = "TeleOp")
class MainTeleOp : BaseOpMode() {
    override fun setup() {
        robot.colorCone?.pipeline = ConePositionPipeline()
    }

    enum class AutoAction {
        GRAB, ALIGN
    }

    private var autoAction: AutoAction? = null

    override fun preRunLoop() {
        // Move the mechanism to the grab position.
        gp2.dpadDown.onActivate = {
            robot.currentLinkState = TheLegend.LinkState.SNIFF
        }

        // Move the mechanism to the rest position.
        gp2.dpadLeft.onActivate = {
            robot.currentLinkState = TheLegend.LinkState.REST
        }

        // Move the mechanism to the cap position.
        gp2.dpadUp.onActivate = {
            robot.currentLinkState = TheLegend.LinkState.CAP
        }
/*
        gp2.x.onActivate = {
            if (robot.currentLinkState == TheLegend.LinkState.SNIFF) {
                ConePositionPipeline.allianceColor = ConePositionPipeline.AllianceColor.BLUE
                //autoAction = AutoAction.ALIGN
                opModeType = OpModeType.Autonomous
            }
        }

        gp2.b.onActivate = {
            if (robot.currentLinkState == TheLegend.LinkState.SNIFF) {
                ConePositionPipeline.allianceColor = ConePositionPipeline.AllianceColor.RED
                //autoAction = AutoAction.ALIGN
                opModeType = OpModeType.Autonomous
            }
        }*/

        gp2.rightTrigger.onActivate = {
            autoAction = AutoAction.GRAB
            opModeType = OpModeType.Autonomous
        }

        // Toggle the gripper when A is pressed.
        listOf(gp1, gp2).forEach {
            it.a.onActivate = {
                robot.clumsyClaw?.let { clumsyClaw ->
                    clumsyClaw.gripper =
                        if (
                            robot.clumsyClaw?.gripper == ClumsyClaw.GripperPosition.OPEN
                            || robot.currentLinkState == TheLegend.LinkState.REST
                        ) ClumsyClaw.GripperPosition.CLOSED
                        else ClumsyClaw.GripperPosition.OPEN
                }
            }
        }
    }

    override fun runLoop() {
        robot.liftyLinkage?.let {
            it.targetPosition -= 0.03 * gp2.leftStickY.correctedValue
        }

        if (robot.liftyLinkage?.isAboveMid == true && robot.currentLinkState != TheLegend.LinkState.SNIFF) {
            robot.liftyLinkage?.lockedAboveMid = true
        }

        robot.nightmareSlide?.adjustment = (-gp2.rightStickY.rawValue() + 1.0) / 2.0
        /*robot.turret?.position = (-gp2.rightStickX.rawValue().toDouble()).let { value ->
            if (value >= 0) Range.scale(value, 0.0, 1.0, Turret.center, 1.0)
            else Range.scale(value, -1.0, 0.0, 0.0, Turret.center)
        }*/

        when (opModeType ?: OpModeType.TeleOp) {
            OpModeType.TeleOp ->
                // Moves the robot based on the GP1 left stick
                robot.zelda?.setWeightedDrivePower(
                    Pose2d( // left stick X
                        -gp1.leftStickY.correctedValue * Range.scale(
                            gp1.rightTrigger.correctedValue.toDouble(),
                            0.0,
                            1.0,
                            drivetrainSpeed,
                            1.0
                        ),  // left sick Y
                        -gp1.leftStickX.correctedValue * Range.scale(
                            gp1.rightTrigger.correctedValue.toDouble(),
                            0.0,
                            1.0,
                            drivetrainSpeed,
                            1.0
                        ),  // right stick X (rotation)
                        -gp1.rightStickX.correctedValue * Range.scale(
                            gp1.rightTrigger.correctedValue.toDouble(),
                            0.0,
                            1.0,
                            drivetrainRotationSpeed,
                            1.0
                        )
                    )
                )
            OpModeType.Autonomous -> {
                if (gamepad1.y || gamepad2.y) autoAction = null

                when (autoAction) {
                    AutoAction.GRAB -> {
                        robot.turret?.position = Turret.center

                        if (robot.clumsyClaw?.sensor?.close == false){
                            robot.zelda?.setWeightedDrivePower(Pose2d(autoForwardSpeed))
                        } else {
                            robot.zelda?.setWeightedDrivePower(Pose2d())
                            robot.clumsyClaw?.gripper = ClumsyClaw.GripperPosition.CLOSED
                            opModeType = OpModeType.TeleOp
                        }
                    }
                    AutoAction.ALIGN -> {
                        val coneDirection = (robot.colorCone?.pipeline as? ConePositionPipeline)!!.coneDirection

                        if (kotlin.math.abs(coneDirection) > .1) {
                            robot.nightmareSlide?.adjustment = -1.0
                            robot.liftyLinkage?.targetPosition = .2
                            robot.waitFor { (robot.liftyLinkage?.currentPosition ?: 1.0) > .15 }

                            robot.zelda?.setWeightedDrivePower(
                                Pose2d(
                                    0.0,
                                    0.0,
                                    kotlin.math.sign(
                                        coneDirection
                                    ) * autoRotationSpeed
                                )
                            )
                        } else {
                            autoAction = null
                        }
                    }
                    else -> {
                        opModeType = OpModeType.TeleOp
                    }
                }
            }
        }
    }

    companion object {
        @JvmField var drivetrainSpeed = .4
        @JvmField var drivetrainRotationSpeed = .3
        @JvmField var autoRotationSpeed = -.08
        @JvmField var autoForwardSpeed = .08
    }
}