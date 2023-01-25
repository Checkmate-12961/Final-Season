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
package org.firstinspires.ftc.teamcode.robot.abstracts

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robot.TheLegend

/**
 * Basic OpMode class that all OpModes should extend
 */
abstract class BaseOpMode : LinearOpMode() {
    protected lateinit var robot: TheLegend
    protected lateinit var gp1: SuperController
    protected lateinit var gp2: SuperController

    private var startTime = 0.0

    private var loopStartTime = 0.0
    private val mspt: Double get() = (runtime - loopStartTime) * 1000
    private val tps: Double get() = 1000.0 / mspt

    /**
     * Type of the op mode.
     */
    protected enum class OpModeType {
        TeleOp, Autonomous
    }

    /**
     * Type of this op mode.
     */
    @JvmField protected var opModeType: OpModeType? = null

    /**
     * Runs before the hardware initializes.
     */
    open fun preSetup() {}

    /**
     * Runs when the OpMode initializes.
     */
    open fun setup() {}

    /**
     * Runs in a loop after the op mode is initialized.
     */
    open fun setupLoop() {}

    /**
     * Runs once before the loop starts.
     */
    open fun preRunLoop() {}

    /**
     * Main OpMode loop, automatically updates the robot.
     */
    open fun runLoop() {}

    /**
     * Runs when the OpMode is stopped.
     */
    open fun cleanup() {}

    /**
     * Implements the above behavior.
     */
    final override fun runOpMode() {
        if (this.javaClass.getAnnotation(TeleOp::class.java) != null) {
            opModeType = OpModeType.TeleOp
        } else if (this.javaClass.getAnnotation(Autonomous::class.java) != null) {
            opModeType = OpModeType.Autonomous
        }
        preSetup()
        robot = TheLegend(hardwareMap)
        gp1 = SuperController(gamepad1)
        gp2 = SuperController(gamepad2)
        setup()
        while (!isStarted) {
            setupLoop()
            updateTelemetry()
        }
        robot.preLoop()
        preRunLoop()
        startTime = runtime
        while (opModeIsActive() && !isStopRequested) {
            if (opModeType == OpModeType.TeleOp) {
                gp1.update()
                gp2.update()
            }
            robot.update()
            runLoop()
            updateTelemetry()
            loopStartTime = runtime
        }
        robot.cleanup()
        cleanup()
    }

    private fun updateTelemetry() {
        // Print stuff to telemetry
        if (opModeIsActive()) {
            telemetry.addData("runtime", "%.2f seconds", runtime - startTime)
            telemetry.addData("tps", "%.2f", tps)
            telemetry.addData("mspt", "%.2f", mspt)
        }

        robot.subsystems.list.forEach {
            telemetry.addLine("\n${it.tag}")
            it.generateTelemetry(telemetry)
        }
        telemetry.update()
    }
}