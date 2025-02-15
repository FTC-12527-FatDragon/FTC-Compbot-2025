package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

import java.util.LinkedList;
import java.util.List;

public class AutomaticFeedforwardTuning extends LinearOpMode {
    public static double FF_RAMP_RATE = 0.1;
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    private SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        velocitySamples.clear();
        voltageSamples.clear();
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                )
        );
    }
}
