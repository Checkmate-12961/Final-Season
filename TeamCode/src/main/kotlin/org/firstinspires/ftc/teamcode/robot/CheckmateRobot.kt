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
package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractRobot
import org.firstinspires.ftc.teamcode.robot.subsystems.ColorCone
import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.robot.util.LynxModuleUtil

/**
 * Checkmate robot class to access subsystems.
 *
 * @constructor
 * Enables Lynx caching. Registers default subsystems.
 *
 * @param hardwareMap Passed in from [org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode].
 */
class CheckmateRobot(hardwareMap: HardwareMap) : AbstractRobot() {
    override val tag = "CheckmateRobot"

    /**
     * Access the [Drivetrain] subsystem from the registry.
     */
    val drivetrain: Drivetrain
        get() = subsystems["Drivetrain"] as Drivetrain

    /**
     * Access the [ColorCone] subsystem
     */
    val colorCone: ColorCone
        get() = subsystems["ColorCone"] as ColorCone

    init {
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap)
        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        // Set up the drivetrain
        subsystems.register(Drivetrain(hardwareMap))

        // Set up the camera (ColorCone)
        subsystems.register(ColorCone(hardwareMap))
    }
}