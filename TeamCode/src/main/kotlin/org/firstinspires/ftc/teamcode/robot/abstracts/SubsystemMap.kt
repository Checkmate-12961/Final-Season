package org.firstinspires.ftc.teamcode.robot.abstracts

import org.firstinspires.ftc.teamcode.robot.abstracts.exceptions.SubsystemException

/**
 * Custom map to store subsystems.
 *
 * @property tag Callback to get the tag for logging.
 */
class SubsystemMap(private val tag: () -> String) {
    /**
     * Please do not directly access this class. It is much safer to use [register] and [unregister].
     */
    val list: MutableList<AbstractSubsystem> = mutableListOf()

    /**
     * Get a subsystem.
     *
     * @param S Type of subsystem to get.
     *
     * @return The subsystem (or null if not registered).
     */
    inline fun <reified S : AbstractSubsystem> get() = list.filterIsInstance<S>().firstOrNull()

    /**
     * Register a subsystem.
     *
     * @param subsystem Subsystem to register.
     */
    inline fun <reified S : AbstractSubsystem> register(subsystem: S) {
        if (list.filterIsInstance<S>().isNotEmpty()) {
            throw SubsystemException("Failed to register already registered subsystem: $subsystem")
        } else {
            list += subsystem
        }
    }

    /**
     * De-register a subsystem.
     *
     * @param S Type of the subsystem to de-register.
     *
     * @return Returns true if a subsystem was unregistered.
     */
    inline fun <reified S : AbstractSubsystem> unregister(): Boolean {
        var removed = false
        list.filterIsInstance<S>().forEach { subsystem ->
            subsystem.cleanup()
            list.remove(subsystem)
            removed = true
        }
        return removed
    }
}