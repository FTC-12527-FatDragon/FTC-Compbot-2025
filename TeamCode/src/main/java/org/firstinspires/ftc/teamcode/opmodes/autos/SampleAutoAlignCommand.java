package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TrajectoryManager;
import org.firstinspires.ftc.teamcode.utils.Pose2dHelperClass;

public class SampleAutoAlignCommand extends CommandBase {
  private final SampleMecanumDrive drive;
  private final Vision vision;
  private final Telemetry telemetry;
  private TrajectorySequence trajectorySequence;
  private boolean isTargetVisibleWhenStart = true;
  private int flag = 0;

  public SampleAutoAlignCommand(SampleMecanumDrive drive, Vision vision, Telemetry telemetry) {
    this.drive = drive;
    this.vision = vision;
    this.telemetry = telemetry;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    isTargetVisibleWhenStart = vision.isTargetVisible();
    flag += 1;
    telemetry.addData("isVisibleWhenStart", isTargetVisibleWhenStart);

    Pose2d currentPoseRelativeToField = drive.getPoseEstimate();
    telemetry.addData("Current Pose", currentPoseRelativeToField);
    Pose2d targetPoseRelativeToRobot =
        new Pose2dHelperClass(vision.getDistance() / 25.4, -vision.getStrafeOffset() / 25.4, 0)
            .toPose2d();
    telemetry.addData("Target Robot Pose", targetPoseRelativeToRobot);

    // Transform target pose from robot-relative to field-relative coordinates
    double fieldX =
        currentPoseRelativeToField.getX()
            + (targetPoseRelativeToRobot.getX() * Math.cos(currentPoseRelativeToField.getHeading())
                - targetPoseRelativeToRobot.getY()
                    * Math.sin(currentPoseRelativeToField.getHeading()));
    double fieldY =
        currentPoseRelativeToField.getY()
            + (targetPoseRelativeToRobot.getX() * Math.sin(currentPoseRelativeToField.getHeading())
                + targetPoseRelativeToRobot.getY()
                    * Math.cos(currentPoseRelativeToField.getHeading()));
    double fieldHeading =
        currentPoseRelativeToField.getHeading() + 0; // We move our claw instead of robot heading

    Pose2d targetPoseRelativeToField = new Pose2d(fieldX, fieldY, fieldHeading);
    telemetry.addData("Target Field Pose", targetPoseRelativeToField);
    telemetry.addData("flag", flag);
    if (isTargetVisibleWhenStart) {
      trajectorySequence =
          TrajectoryManager.trajectorySequenceBuilder(currentPoseRelativeToField)
              .lineToLinearHeading(targetPoseRelativeToField)
              .build();
    } else {
        cancel();
    }
    drive.followTrajectorySequenceAsync(trajectorySequence);
  }

  @Override
  public void execute() {
    drive.update();
  }

  @Override
  public void end(boolean interrupted) {
    isTargetVisibleWhenStart = true;
  }

  @Override
  public boolean isFinished() {
    if (!isTargetVisibleWhenStart) {
      return true;
    }
    return !drive.isBusy();
  }
}
