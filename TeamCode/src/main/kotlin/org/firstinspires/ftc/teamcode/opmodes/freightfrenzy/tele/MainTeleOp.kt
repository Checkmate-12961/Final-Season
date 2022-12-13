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
import org.firstinspires.ftc.teamcode.robot.subsystems.LiftyLinkage

@Config
@TeleOp(name = "TeleOp")
class MainTeleOp : BaseOpMode() {
    //private var position = 0

    override fun runLoop() {
        gp1.leftTrigger.whileActive = { if (!gp1.leftBumper.active) robot.liftyLinkage.action(LiftyLinkage.Action.DOWN) }
        gp1.leftBumper.whileActive = { robot.liftyLinkage.action(LiftyLinkage.Action.UP) }

        gp1.leftBumper.whileInactive = { if (!gp1.leftTrigger.active) robot.liftyLinkage.action(LiftyLinkage.Action.HOLD) }

        // Move the mechanism to the grab position.
        // If the slide is extended, do nothing.
        gp1.dpadDown.onActivate = {
            if (
                robot.clumsyClaw.pivot != ClumsyClaw.PivotPosition.CAP
                && robot.clumsyClaw.slide != ClumsyClaw.SlidePosition.EXTENDED
            ) {
                robot.clumsyClaw.wrist = ClumsyClaw.WristPosition.GRAB
                robot.clumsyClaw.pivot = ClumsyClaw.PivotPosition.GRAB
            }
        }

        // Move the mechanism to the rest position.
        // If the slide is extended, do nothing.
        gp1.dpadLeft.onActivate = {
            if (robot.clumsyClaw.slide != ClumsyClaw.SlidePosition.EXTENDED) {
                robot.clumsyClaw.wrist = ClumsyClaw.WristPosition.REST
                robot.clumsyClaw.pivot = ClumsyClaw.PivotPosition.REST
            }
        }

        // Move the mechanism to the rest position with the wrist in the grab position
        // If the pivot is not in the rest position, do nothing.
        gp1. dpadRight.onActivate = {
            if (robot.clumsyClaw.pivot != ClumsyClaw.PivotPosition.CAP) {
                robot.clumsyClaw.wrist = ClumsyClaw.WristPosition.GRAB
                robot.clumsyClaw.pivot = ClumsyClaw.PivotPosition.REST
            }
        }

        // Move the mechanism to the cap position.
        // If anything is not in the rest position, do nothing.
        gp1.dpadUp.onActivate = {
            if (
                robot.clumsyClaw.pivot != ClumsyClaw.PivotPosition.GRAB
                && robot.clumsyClaw.slide != ClumsyClaw.SlidePosition.EXTENDED
                && robot.clumsyClaw.wrist != ClumsyClaw.WristPosition.GRAB
            ) {
                robot.clumsyClaw.pivot = ClumsyClaw.PivotPosition.CAP
                robot.clumsyClaw.wrist = ClumsyClaw.WristPosition.CAP
            }
        }

        // Toggle the gripper when B is pressed.
        // If the pivot is in the rest position, do nothing.
        gp1.b.onActivate = {
            if (robot.clumsyClaw.wrist != ClumsyClaw.WristPosition.REST) {
                robot.clumsyClaw.gripper =
                    if (
                        robot.clumsyClaw.gripper == ClumsyClaw.GripperPosition.OPEN
                    ) ClumsyClaw.GripperPosition.CLOSED
                    else ClumsyClaw.GripperPosition.OPEN
            }
        }

        // Contract the slide when the right bumper is pressed.
        // If the pivot is NOT in the rest position, do nothing.
        gp1.rightBumper.onActivate = {
            if (
                robot.clumsyClaw.pivot != ClumsyClaw.PivotPosition.GRAB
                && robot.clumsyClaw.wrist != ClumsyClaw.WristPosition.GRAB
            ) {
                robot.clumsyClaw.slide = ClumsyClaw.SlidePosition.CONTRACTED
            }
        }

        // Extend the slide when the right bumper is pressed.
        // If the pivot is NOT in the rest position, do nothing.
        gp1.rightTrigger.onActivate = {
            if (
                robot.clumsyClaw.pivot != ClumsyClaw.PivotPosition.GRAB
                && robot.clumsyClaw.wrist != ClumsyClaw.WristPosition.GRAB
            ) {
                robot.clumsyClaw.slide = ClumsyClaw.SlidePosition.EXTENDED
            }
        }

        when (opModeType) {
            OpModeType.TeleOp ->
                // Moves the robot based on the GP1 left stick
                robot.longSchlong.setWeightedDrivePower(
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