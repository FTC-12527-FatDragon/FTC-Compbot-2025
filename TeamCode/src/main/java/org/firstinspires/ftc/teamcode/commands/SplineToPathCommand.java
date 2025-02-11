package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TrajectoryManager;

public class SplineToPathCommand extends CommandBase {
    private final SampleMecanumDrive drive;
    private double endHeading;
    private Pose2d goalPose;
    private TrajectorySequence trajectorySequence;
    private boolean isBack;

    public SplineToPathCommand(SampleMecanumDrive drive, Pose2d goalPose, double endHeading, boolean back) {
        this.drive = drive;
        this.goalPose = goalPose;
        this.endHeading = endHeading;
        isBack = back;
    }

    public SplineToPathCommand(SampleMecanumDrive drive, Pose2d goalPose, double endHeading) {
        this(drive, goalPose, endHeading, false);
    }

    public SplineToPathCommand(SampleMecanumDrive drive, Pose2d goalPose) {
        this(drive, goalPose, 0, false);
    }

    @Override
    public void initialize() {
        trajectorySequence =
                TrajectoryManager.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToLinearHeading(goalPose, endHeading)
                        .build();
        drive.followTrajectorySequenceAsync(trajectorySequence);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public boolean isFinished() {
        return !drive.isBusy();
    }
}
