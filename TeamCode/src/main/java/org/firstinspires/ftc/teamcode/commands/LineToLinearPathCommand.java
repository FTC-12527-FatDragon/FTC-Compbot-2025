package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TrajectoryManager;

public class LineToLinearPathCommand extends CommandBase {
  private final SampleMecanumDrive drive;
  private Pose2d goalPose;
  private Pose2d currentPose;
  private TrajectorySequence trajectorySequence;
  private boolean shouldWithinRange;
  private double maxAngAccel;

  public LineToLinearPathCommand(SampleMecanumDrive drive, Pose2d goalPose) {
    this(drive, goalPose, false, DriveConstants.MAX_ANG_ACCEL);
  }

  public LineToLinearPathCommand(SampleMecanumDrive drive, Pose2d goalPose, double maxAngAccel) {
    this(drive, goalPose, false, maxAngAccel);
  }

  public LineToLinearPathCommand(
      SampleMecanumDrive drive, Pose2d goalPose, boolean shouldWithinRange, double maxAngAccel) {
    this.drive = drive;
    this.goalPose = goalPose;
    this.shouldWithinRange = shouldWithinRange;
    this.maxAngAccel = maxAngAccel;
  }

  @Override
  public void initialize() {
    currentPose = new Pose2d();
    trajectorySequence =
        TrajectoryManager.trajectorySequenceBuilder(drive.getPoseEstimate(), maxAngAccel)
            .lineToLinearHeading(goalPose)
            .build();
    drive.followTrajectorySequenceAsync(trajectorySequence);
  }

  private boolean isWithinRange() {
    return MathUtil.isNear(goalPose.getX(), currentPose.getX(), DriveConstants.xPoseError)
        && MathUtil.isNear(goalPose.getY(), currentPose.getY(), DriveConstants.yPoseError)
        && MathUtil.isNear(
            goalPose.getHeading(), currentPose.getHeading(), DriveConstants.headingPoseError);
  }

  @Override
  public void execute() {
    drive.update();
    currentPose = drive.getPoseEstimate();
  }

  @Override
  public void end(boolean interrupted) {
    drive.breakFollowing(true);
  }

  @Override
  public boolean isFinished() {
    if (shouldWithinRange) {
      return isWithinRange();
    }
    return !drive.isBusy();
  }
}
