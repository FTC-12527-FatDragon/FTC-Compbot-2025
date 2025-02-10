package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TrajectoryManager;

@Config
public class AutoPathCommand extends CommandBase {
    public static double tangent = -90;
  private final SampleMecanumDrive drive;
  private Pose2d goalPose;
  private TrajectorySequence trajectorySequence;

  public AutoPathCommand(SampleMecanumDrive drive, Pose2d goalPose) {
    this.drive = drive;
    this.goalPose = goalPose;
  }

  @Override
  public void initialize() {
    trajectorySequence =
        TrajectoryManager.trajectorySequenceBuilder(drive.getPoseEstimate())
            .splineToSplineHeading(goalPose, -90)
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
