package org.firstinspires.ftc.teamcode.robot.abstracts

class DefaultSubsystem : AbstractSubsystem {
    override val tag = this.javaClass.simpleName
    override val subsystems = SubsystemMap { tag }
}