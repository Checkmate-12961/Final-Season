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

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.robot.util.Encoder

/**
 * Class to manage names of hardware connected to the robot
 */
class HardwareNames {
    /**
     * Contains hardware info for motors.
     *
     * @property id Hardware ID defined in the config.
     * @property reverse Whether the motor should be reversed.
     *
     * @property get Get a [DcMotorEx] with this id from the [HardwareMap].
     */
    enum class Motors(private val id: String, private val reverse: Boolean) {
        // Drivetrain
        RIGHT_FRONT("rightFront", false),
        RIGHT_REAR("rightRear", false),
        LEFT_FRONT("leftFront",true),
        LEFT_REAR("leftRear", true),

        // Lift system
        LIFTY_LINKAGE_A("motor3", false),
        LIFTY_LINKAGE_B("motor2", true);

        fun get(hardwareMap: HardwareMap): DcMotorEx {
            val motor = hardwareMap.get(DcMotorEx::class.java, id)
            if (reverse) {
                motor.direction = DcMotorSimple.Direction.REVERSE
            }
            return motor
        }
    }

    /**
     * Contains hardware info for encoders.
     *
     * @property id Hardware ID defined in the config.
     * @property reverse Whether the encoder should be reversed.false
     *
     * @property get Get an [Encoder] with this id from the [HardwareMap].
     */
    enum class Encoders(val id: String, val reverse: Boolean) {
        FRONT("motor0", false),
        CENTER("motor2", true),
        REAR("motor1", true);

        fun get(hardwareMap: HardwareMap): Encoder {
            val encoder = Encoder(hardwareMap.get(DcMotorEx::class.java, id))
            if (reverse) {
                encoder.direction = Encoder.Direction.REVERSE
            }
            return encoder
        }
    }

    /**
     * Contains hardware info for servos.
     *
     * @property id Hardware ID defined in the config.
     * @property reverse Whether the servo should be reversed.
     *
     * @property get Get a [Servo] with this id from the [HardwareMap].
     */
    enum class Servos(val id: String, val reversed: Boolean) {
        CLUMSY_CLAW("clumsyClaw", false);

        fun get(hardwareMap: HardwareMap): Servo {
            val servo = hardwareMap.get(Servo::class.java, id)
            if (reversed) {
                servo.direction = Servo.Direction.REVERSE
            }
            return servo
        }
    }

    /**
     * Contains hardware info for continuous rotation servos.
     *
     * @property id Hardware ID defined in the config.
     * @property reverse Whether the continuous rotation servo should be reversed.
     *
     * @property get Get a [CRServo] with this id from the [HardwareMap].
     */
    enum class CRServos(val id: String, val reverse: Boolean) {
        ;

        fun get(hardwareMap: HardwareMap): CRServo {
            val crServo = hardwareMap.get(CRServo::class.java, id)
            if (reverse) {
                crServo.direction = DcMotorSimple.Direction.REVERSE
            }
            return crServo
        }
    }

    /**
     * Contains hardware info for cameras.
     *
     * @property id Hardware ID defined in the config.
     *
     * @property get Get a [WebcamName] with this id from the [HardwareMap].
     */
    enum class Cameras(val id: String) {
        WEBCAM("webcam");

        fun get(hardwareMap: HardwareMap): WebcamName {
            return hardwareMap.get(WebcamName::class.java, id)
        }
    }

    /**
     * Contains hardware info for digital I/O ports.
     *
     * @property id Hardware ID defined in the config.
     * @property mode The mode of the digital I/O port.
     *
     * @property get Get a [DigitalChannel] with this id from the [HardwareMap].
     */
    enum class DigitalChannels(val id: String, val mode: DigitalChannel.Mode) {
        ;

        fun get(hardwareMap: HardwareMap): DigitalChannel {
            val digitalChannel = hardwareMap.get(DigitalChannel::class.java, this.id)
            digitalChannel.mode = this.mode
            return digitalChannel
        }
    }
}