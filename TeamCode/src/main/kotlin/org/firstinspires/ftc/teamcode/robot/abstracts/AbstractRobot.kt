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

/**
 * Class that all robots need to extend
 */
abstract class AbstractRobot {
    /**
     * String tag for logging.
     */
    protected abstract val tag: String

    /**
     * Storage for added subsystems.
     */
    val subsystems = SubsystemMap{ tag }

    /**
     * Runs once before the loop.
     */
    fun preLoop() {
        recursiveAction(subsystems) {
            it.preLoop()
        }
    }

    /**
     * Updates all subsystems.
     */
    fun update() {
        recursiveAction(subsystems) {
            it.loop()
        }
    }

    /**
     * Runs cleanup() on all subsystems.
     */
    fun cleanup() {
        recursiveAction(subsystems) {
            it.cleanup()
        }
    }

    /**
     * Recursively does actions on subsystems.
     *
     * @param subsystemMap The initial [SubsystemMap] to recurse from.
     */
    fun recursiveAction(
        subsystemMap: SubsystemMap, action: (AbstractSubsystem) -> Unit
    ) {
        for (subsystem in subsystemMap.list) {
            action.invoke(subsystem)
            recursiveAction(subsystem.subsystems, action)
        }
    }
}