package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TrajectoryManager;

@Config
public class LineToLinearPathCommand extends CommandBase {
  private final SampleMecanumDrive drive;
  private Pose2d goalPose;
  private TrajectorySequence trajectorySequence;
  private boolean isBack;

  public LineToLinearPathCommand(SampleMecanumDrive drive, Pose2d goalPose) {
    this.drive = drive;
    this.goalPose = goalPose;
    isBack = false;
  }

  public LineToLinearPathCommand(SampleMecanumDrive drive, Pose2d goalPose, boolean isBack) {
    this.drive = drive;
    this.goalPose = goalPose;
    this.isBack = isBack;
  }

  @Override
  public void initialize() {
    if(!isBack) {
      trajectorySequence =
              TrajectoryManager.trajectorySequenceBuilder(drive.getPoseEstimate())
                      .lineToLinearHeading(goalPose)
                      .build();
      drive.followTrajectorySequenceAsync(trajectorySequence);
    }
    else {
      trajectorySequence =
              TrajectoryManager.trajectorySequenceBuilder(drive.getPoseEstimate())
                      .forward(-12)
                      .lineToLinearHeading(goalPose)
                      .build();
      drive.followTrajectorySequenceAsync(trajectorySequence);
    }

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
