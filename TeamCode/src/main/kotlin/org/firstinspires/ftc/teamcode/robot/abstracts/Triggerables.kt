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

import com.qualcomm.robotcore.util.Range

class Triggerables {
    /**
     * Enum to store the direction the triggerable is triggered in
     */
    enum class TriggerDirection {
        UP, OFF, DOWN;
    }

    /**
     * Triggerable class to manage callbacks and states
     */
    abstract class AbstractTriggerable {
        /**
         * Whether or not the triggerable is currently active
         */
        var active: Boolean = false
            protected set

        @JvmField
        var onToggle: (() -> Unit)? = null

        /**
         * Callback for when the button is pressed
         */
        @JvmField
        var onActivate: (() -> Unit)? = null

        /**
         * Callback for while the button is pressed
         */
        @JvmField
        var whileActive: (() -> Unit)? = null

        /**
         * Callback for when the button is released
         */
        @JvmField
        var onDeactivate: (() -> Unit)? = null

        /**
         * Callback for while the triggerable is deactivated
         */
        @JvmField
        var whileInactive: (() -> Unit)? = null

        /**
         * Updates the triggerable state
         */
        abstract fun update()
    }

    /**
     * Button class to manage callbacks and states
     */
    abstract class AnalogTriggerable : AbstractTriggerable() {
        /**
         * The threshold for triggerable activation
         */
        abstract var activationThreshold: Double
    }

    /**
     * Button class to manage callbacks and states
     */
    class ControllerButton(function: () -> Boolean) : AbstractTriggerable() {
        /**
         * Get the value of the button from the controller
         */
        @JvmField
        val rawPressed: () -> Boolean = function

        /**
         * Updates the button state
         */
        override fun update() {
            // if it's pressed
            if (rawPressed()) {
                // but it doesn't know
                if (!active) {
                    // tell it
                    active = true
                    // then run the activate callback (if it's set)
                    onActivate?.invoke()
                    onToggle?.invoke()
                } else {
                    // otherwise run the normal active callback
                    whileActive?.invoke()
                }
            }
            // if it isn't pressed
            else if (!rawPressed()) {
                // but it doesn't know
                if (active) {
                    // tell it
                    active = false
                    // and run the deactivate callback (if it's set)
                    onDeactivate?.invoke()
                    onToggle?.invoke()
                } else {
                    // otherwise run the normal inactive callback
                    whileInactive?.invoke()
                }
            }
        }
    }

    /**
     * Trigger class to manage callbacks and states
     */
    class ControllerTrigger(function: () -> Float) : AnalogTriggerable() {
        /**
         * The threshold for triggerable activation
         */
        override var activationThreshold = 0.4

        /**
         * Get the value of the trigger from the controller
         */
        @JvmField
        val rawValue: () -> Float = function

        /**
         * Get corrected value based on the activation threshold
         */
        var correctedValue = 0F
            private set

        /**
         * Updates the trigger state
         */
        override fun update() {
            // if it's over the threshold
            if (rawValue() >= activationThreshold) {
                // set the corrected value
                correctedValue = Range.scale(rawValue().toDouble(), activationThreshold, 1.0, 0.0, 1.0).toFloat()
                // and it doesn't know it's active
                if (!active) {
                    // set it as active
                    active = true
                    // and run the callback (if it's set)
                    onActivate?.invoke()
                    onToggle?.invoke()
                } else {
                    // then run the repeat callback (if it's set)
                    whileActive?.invoke()
                }
            }
            // if it's under the threshold
            else if (rawValue() < activationThreshold) {
                // zero the corrected value
                correctedValue = 0F
                // and it thinks it's active
                if (active) {
                    // set it as inactive
                    active = false
                    // and run the callback (if it's set)
                    onDeactivate?.invoke()
                    onToggle?.invoke()
                } else {
                    // then run the repeat callback (if it's set)
                    whileInactive?.invoke()
                }
            }
        }

    }

    /**
     * Stick class to manage callbacks and states
     */
    class ControllerStick(function: () -> Float) : AnalogTriggerable() {
        /**
         * The threshold for stick activation
         */
        override var activationThreshold: Double = 0.3

        /**
         * Get the value of the stick from the controller
         */
        @JvmField
        val rawValue: () -> Float = function

        /**
         * What direction the stick is currently active in
         */
        var state = TriggerDirection.OFF
            private set

        /**
         * What the value of the stick is if it's corrected by the threshold
         */
        var correctedValue = 0.0F
            private set

        /**
         * Callback for when the stick is activated negatively
         */
        @JvmField
        var onActivateNeg: (() -> Unit)? = null

        /**
         * Callback for when the stick is deactivated negatively
         */
        @JvmField
        var onDeactivateNeg: (() -> Unit)? = null

        /**
         * Callback for while the stick is active negatively
         */
        @JvmField
        var whileActiveNeg: (() -> Unit)? = null

        /**
         * Updates the stick state
         */
        override fun update() {
            // update the threshold-corrected value
            correctedValue = if (rawValue() >= activationThreshold) {
                Range.scale(rawValue().toDouble(), activationThreshold, 1.0, 0.0, 1.0).toFloat()
            } else if (rawValue() <= -activationThreshold) {
                -Range.scale(-rawValue().toDouble(), activationThreshold, 1.0, 0.0, 1.0).toFloat()
            } else {
                0f
            }


            when (state) {
                // if it thinks it's up
                TriggerDirection.UP -> {
                    // but it isn't
                    if (rawValue() < activationThreshold) {
                        // tell it
                        active = false
                        state = TriggerDirection.OFF
                        // and run the deactivate callback (if it's set)
                        onDeactivate?.invoke()
                        onToggle?.invoke()
                    } else {
                        // otherwise run the normal up callback (if it's set)
                        whileActive?.invoke()
                    }
                }
                // if it thinks it's centered
                TriggerDirection.OFF -> {
                    // but it's up
                    if (rawValue() >= activationThreshold) {
                        // tell it
                        active = true
                        state = TriggerDirection.UP
                        // and run the activate callback (if it's set)
                        onActivate?.invoke()
                        onToggle?.invoke()
                    }
                    // but it's down
                    else if (rawValue() <= -activationThreshold) {
                        // tell it
                        active = true
                        state = TriggerDirection.DOWN
                        // and run the negative active callback (if it's set)
                        onActivateNeg?.invoke()
                        onToggle?.invoke()
                    } else {
                        // otherwise run the normal off callback (if it's set)
                        whileInactive?.invoke()
                    }
                }
                // if it thinks it's down
                TriggerDirection.DOWN -> {
                    // but it isn't
                    if (rawValue() > -activationThreshold) {
                        // tell it
                        active = false
                        state = TriggerDirection.OFF
                        // and run the negative deactivation callback (if it's set)
                        onDeactivateNeg?.invoke()
                        onToggle?.invoke()
                    } else {
                        // otherwise run the normal down callback (if it's set)
                        whileActiveNeg?.invoke()
                    }
                }
            }
        }

    }
}