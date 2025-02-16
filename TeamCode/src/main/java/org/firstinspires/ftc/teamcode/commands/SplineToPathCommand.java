package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TrajectoryManager;

public class SplineToPathCommand extends CommandBase {
  private final SampleMecanumDrive drive;
  private double endHeading;
  private Pose2d goalPose;
  private Pose2d currentPose;
  private TrajectorySequence trajectorySequence;
  private boolean isBack;

  public SplineToPathCommand(
      SampleMecanumDrive drive, Pose2d goalPose, double endHeading, boolean back) {
    addRequirements(drive);
    this.drive = drive;
    this.goalPose = goalPose;
    this.endHeading = endHeading;
    isBack = back;
  }

  public SplineToPathCommand(SampleMecanumDrive drive, Pose2d goalPose, double endHeading) {
    this(drive, goalPose, endHeading, false);
  }

  public SplineToPathCommand(SampleMecanumDrive drive, Pose2d goalPose, boolean back) {
    this(drive, goalPose, 0, back);
  }

  @Override
  public void initialize() {
    currentPose = drive.getPoseEstimate();
    if (!isBack) {
      trajectorySequence =
          TrajectoryManager.trajectorySequenceBuilder(drive.getPoseEstimate())
              .splineToLinearHeading(goalPose, endHeading)
              .build();
      drive.followTrajectorySequenceAsync(trajectorySequence);
    } else {
      Trajectory trajectory =
          TrajectoryManager.trajectoryBuilder(drive.getPoseEstimate(), true)
              .splineToLinearHeading(goalPose, Math.toRadians(250))
              .build();
      drive.followTrajectoryAsync(trajectory);
    }
  }

  private boolean isWithinRange() {
    return MathUtil.isNear(goalPose.getX(), currentPose.getX(), DriveConstants.xPoseError)
        && MathUtil.isNear(goalPose.getY(), currentPose.getY(), DriveConstants.yPoseError);
//        && MathUtil.isNear(
//            goalPose.getHeading(), currentPose.getHeading(), DriveConstants.headingPoseError);
  }

  @Override
  public void execute() {
    currentPose = drive.getPoseEstimate();
    drive.update();
  }

  @Override
  public void end(boolean interrupted) {
    drive.breakFollowing(true);
  }

  @Override
  public boolean isFinished() {
//    if (isBack) {
//      return isWithinRange();
//    }
    return !drive.isBusy();
  }
}
