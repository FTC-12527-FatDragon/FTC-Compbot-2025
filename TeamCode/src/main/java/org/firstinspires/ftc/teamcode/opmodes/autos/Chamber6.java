package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.commands.LineToLinearPathCommand;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.MathUtils;
import org.firstinspires.ftc.teamcode.utils.Pose2dHelperClass;

@Config
@Autonomous(name = "Chamber 6", group = "Autos")
public class Chamber6 extends AutoCommandBase {
  public static Pose2d startPose = new Pose2d(27.71, -63.75, Math.toRadians(90));

  public static Pose2dHelperClass S1 = new Pose2dHelperClass(48.591, -54.72, 90);

  public static Pose2dHelperClass S2 = new Pose2dHelperClass(57.5, -54, 90);

  public static Pose2dHelperClass S3 = new Pose2dHelperClass(59, -52, Math.toDegrees(1.2));

  public static Pose2dHelperClass grabSpecPose = new Pose2dHelperClass(35.46, -63.42, 270);

  public static Pose2dHelperClass spec1Pose = new Pose2dHelperClass(4.118, -31.49, 270);

  public static Pose2dHelperClass spec2Pose = new Pose2dHelperClass(3.118, -31.49, 270);

  public static Pose2dHelperClass spec3Pose = new Pose2dHelperClass(2.118, -31.49, 270);

  public static Pose2dHelperClass spec4Pose = new Pose2dHelperClass(1.118, -31.49, 270);

  public static Pose2dHelperClass spec5Pose = new Pose2dHelperClass(0.118, -31.49, 270);

  public static Pose2dHelperClass spec6Pose = new Pose2dHelperClass(-1.118, -31.49, 270);

  public static long delayToUpLift = 300;
  public static long hangToOpenClaw = 500;
  public static long pathToStow = 0;
  public static long pathToExtendSlide = 0;
  public static long liftClawOpenToFold = 300;
  public static long liftUpToOpen = 300;

  Supplier<Command> upLiftArmOpen =
      () ->
          new SequentialCommandGroup(
              liftClaw.setLiftClawServo(LiftClaw.LiftClawState.SCORE_BASKET, liftUpToOpen),
              liftClaw.openClawCommand());

  Supplier<Command> slideExtendCommand =
      () -> new InstantCommand(() -> slide.forwardSlideExtension(350));

  @Override
  public Pose2d getStartPose() {
    return startPose;
  }

  @Override
  public void initializeSuperStructure() {
    drive.breakFollowing(true);
    slide.stow();
    slide.backwardSlideExtension();
    liftClaw.foldLiftArm();
    liftClaw.closeClaw();
  }

  @Override
  public Command runAutoCommand() {
    Supplier<Command> hangSpecimen =
        () -> new InstantCommand(() -> lift.setGoal(Lift.Goal.PRE_HANG));

    Supplier<Command> stowLiftAndArm =
        () ->
            new SequentialCommandGroup(
                liftClaw.setLiftClawServo(LiftClaw.LiftClawState.GRAB_FROM_WALL, 100),
                new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW)));

    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              drive.setPoseEstimate(startPose);
              drive.setCurrentTrajectoryMode(SampleMecanumDrive.TrajectoryMode.MEDIUM);
            }),
        new LineToLinearPathCommand(drive, spec1Pose.toPose2d()),
        turnCommand(100),
        new LineToLinearPathCommand(drive, spec2Pose.toPose2d()),
        turnCommand(100),
        new LineToLinearPathCommand(drive, spec3Pose.toPose2d()),
        turnCommand(100));
  }

  private Command turnCommand(double turnDegrees) {
    return new FunctionalCommand(
        () -> drive.turnAsync(Math.toRadians(turnDegrees)),
        () -> drive.update(),
        (interrupted) -> {},
        () ->
            !drive.isBusy()
                || MathUtils.isNear(
                    turnDegrees,
                    drive.getPoseEstimate().getHeading(),
                    DriveConstants.headingPoseError),
        drive);
  }
}
