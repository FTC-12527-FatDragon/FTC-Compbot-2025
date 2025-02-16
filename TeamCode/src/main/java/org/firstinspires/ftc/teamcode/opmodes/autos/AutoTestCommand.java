package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Test Log")
public class AutoTestCommand extends AutoCommandBase{
    @Override
    public Command runAutoCommand() {
        return new SequentialCommandGroup(
                new WaitCommand(100000000)
        );
    }

    @Override
    public Pose2d getStartPose() {
        return new Pose2d();
    }

    @Override
    public void initializeSuperStructure() {

    }
}
