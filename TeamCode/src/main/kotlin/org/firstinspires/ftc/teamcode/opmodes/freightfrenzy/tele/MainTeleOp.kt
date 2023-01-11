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
package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.tele

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.subsystems.ClumsyClaw
import org.firstinspires.ftc.teamcode.robot.subsystems.T

@Config
@TeleOp(name = "TeleOp")
class MainTeleOp : BaseOpMode() {
    enum class LinkState {
        SNIFF, REST, CAP
    }

    private var currentLinkState = LinkState.REST
        set(value) {
            // Checks
            if (currentLinkState != LinkState.REST && value != LinkState.REST) return
            if (value == LinkState.CAP && !robot.liftyLinkage.isAboveMid) return

            field = value

            when (value) {
                LinkState.SNIFF -> {
                    robot.clumsyClaw.wrist = ClumsyClaw.WristPosition.BIG_EYES
                    robot.clumsyClaw.pivot = ClumsyClaw.PivotPosition.GRAB
                    robot.t.locked = false
                    robot.nightmareSlide.currentFrame = -1
                    robot.liftyLinkage.lockedAboveMid = false
                }
                LinkState.REST -> {
                    robot.t.locked = true
                    if (currentLinkState == LinkState.CAP) {
                        robot.nightmareSlide.currentFrame = 2
                        sleep(300)
                        robot.nightmareSlide.currentFrame = 1
                        sleep(300)
                    }
                    robot.clumsyClaw.gripper = ClumsyClaw.GripperPosition.CLOSED
                    robot.clumsyClaw.wrist = ClumsyClaw.WristPosition.SMALL_EYES
                    robot.clumsyClaw.pivot = ClumsyClaw.PivotPosition.REST
                    robot.nightmareSlide.currentFrame = 0
                    robot.liftyLinkage.lockedAboveMid = false
                }
                LinkState.CAP -> {
                    robot.clumsyClaw.wrist = ClumsyClaw.WristPosition.SMALL_EYES
                    robot.clumsyClaw.pivot = ClumsyClaw.PivotPosition.CAP
                    robot.t.locked = false
                    robot.nightmareSlide.currentFrame = 1
                    robot.liftyLinkage.lockedAboveMid = true

                    sleep(300)
                    robot.nightmareSlide.currentFrame = 2
                    sleep(300)
                    robot.nightmareSlide.currentFrame = 3
                }
            }
        }

    override fun preRunLoop() {
        // Move the mechanism to the grab position.
        gp2.dpadDown.onActivate = {
            currentLinkState = LinkState.SNIFF
        }

        // Move the mechanism to the rest position.
        gp2.dpadLeft.onActivate = {
            currentLinkState = LinkState.REST
        }

        // Move the mechanism to the cap position.
        gp2.dpadUp.onActivate = {
            currentLinkState = LinkState.CAP
        }

        // Toggle the gripper when B is pressed.
        // If the pivot is in the rest position, do nothing.
        listOf(gp1, gp2).forEach {
            it.b.onActivate = {
                robot.clumsyClaw.gripper =
                    if (
                        robot.clumsyClaw.gripper == ClumsyClaw.GripperPosition.OPEN
                        || currentLinkState == LinkState.REST
                    ) ClumsyClaw.GripperPosition.CLOSED
                    else ClumsyClaw.GripperPosition.OPEN
            }
        }
    }

    override fun runLoop() {
        robot.liftyLinkage.targetPosition -= 0.03 * gp2.leftStickY.correctedValue

        robot.nightmareSlide.adjustment = (-gp2.rightStickY.rawValue() + 1.0) / 2.0
        robot.t.position = (-gp2.rightStickX.rawValue().toDouble()).let { value ->
            if (value >= 0) Range.scale(value, 0.0, 1.0, T.center, 1.0)
            else Range.scale(value, -1.0, 0.0, 0.0, T.center)
        }

        when (opModeType) {
            OpModeType.TeleOp ->
                // Moves the robot based on the GP1 left stick
                robot.zelda.setWeightedDrivePower(
                    Pose2d( // left stick X
                        -gp1.leftStickY.correctedValue * Range.scale(
                            gp1.rightTrigger.correctedValue.toDouble(),
                            -1.0,
                            1.0,
                            0.0,
                            1.0
                        ),  // left sick Y
                        -gp1.leftStickX.correctedValue * Range.scale(
                            gp1.rightTrigger.correctedValue.toDouble(),
                            -1.0,
                            1.0,
                            0.0,
                            1.0
                        ),  // right stick X (rotation)
                        -gp1.rightStickX.correctedValue * Range.scale(
                            gp1.rightTrigger.correctedValue.toDouble(),
                            -1.0,
                            1.0,
                            0.0,
                            1.0
                        )
                    )
                )
            OpModeType.Autonomous -> {
                // Replace false here with a check to cancel the sequence
                // if (false) robot.longSchlong.cancelSequence()
                // if (!robot.longSchlong.isBusy) opModeType = OpModeType.TeleOp
            }
            else ->
                // If we end up here, something went horribly wrong.
                // Generally, the best plan of action is to ignore
                //  it and move on.
                opModeType = OpModeType.TeleOp
        }
    }
}