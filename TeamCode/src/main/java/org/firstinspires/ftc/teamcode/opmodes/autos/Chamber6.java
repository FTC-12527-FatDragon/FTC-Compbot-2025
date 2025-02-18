package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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
  public static Pose2d startPose = new Pose2d(18.5, -63.75, Math.toRadians(0));

  public static Pose2dHelperClass S1Start =
      new Pose2dHelperClass(32.06, -38.63, Math.toDegrees(0.7514));
  public static Pose2dHelperClass S1End =
      new Pose2dHelperClass(37.312, -49.514, Math.toDegrees(-0.668));

  public static Pose2dHelperClass S2Start =
      new Pose2dHelperClass(39.9, -35.08, Math.toDegrees(0.482));
  public static Pose2dHelperClass S2End =
      new Pose2dHelperClass(43.94, -49.159, Math.toDegrees(-0.58));

  public static Pose2dHelperClass S3Start =
      new Pose2dHelperClass(49.07, -33.38, Math.toDegrees(0.381));
  public static Pose2dHelperClass S3End =
      new Pose2dHelperClass(50.726, -45.485, Math.toDegrees(-0.757));

  public static Pose2dHelperClass grabSpecPose = new Pose2dHelperClass(35.46, -63.42, 270);

  public static Pose2dHelperClass spec1Pose = new Pose2dHelperClass(4.118, -31.49, 270);

  public static Pose2dHelperClass spec2Pose = new Pose2dHelperClass(3.118, -31.49, 270);

  public static Pose2dHelperClass spec3Pose = new Pose2dHelperClass(2.118, -31.49, 270);

  public static Pose2dHelperClass spec4Pose = new Pose2dHelperClass(1.118, -31.49, 270);

  public static Pose2dHelperClass spec5Pose = new Pose2dHelperClass(0.118, -31.49, 270);

  public static Pose2dHelperClass spec6Pose = new Pose2dHelperClass(-1.118, -31.49, 270);

  public static long delayToUpLift = 100;
  public static long delayToStow = 100;
  public static long hangToOpenClaw = 100;
  public static long pathToGrab = 100;
  public static long liftUpToOpen = 100;
  public static long pathStartToSwipe = 100;
  public static double turnDegrees = 100;

  public static long waitForArm = 200;

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
    drive.setPoseEstimate(startPose);
    drive.setCurrentTrajectoryMode(SampleMecanumDrive.TrajectoryMode.MEDIUM);
    drive.breakFollowing(true);
    slide.stow();
    slide.backwardSlideExtension();
    liftClaw.foldLiftArm();
    liftClaw.closeClaw();
  }

  @Override
  public Command runAutoCommand() {

    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              drive.setPoseEstimate(startPose);
              drive.setCurrentTrajectoryMode(SampleMecanumDrive.TrajectoryMode.MEDIUM);
              slide.stow();
            }),
        new LineToLinearPathCommand(drive, S1Start.toPose2d()),
        slide
            .swipeCommand()
            .alongWith(
                new WaitCommand(pathStartToSwipe)
                    .andThen(new LineToLinearPathCommand(drive, S1End.toPose2d()))),
        new InstantCommand(() -> slide.slideArmUp()),
        new LineToLinearPathCommand(drive, S2Start.toPose2d()),
        slide
            .swipeCommand()
            .alongWith(
                new WaitCommand(pathStartToSwipe)
                    .andThen(new LineToLinearPathCommand(drive, S2End.toPose2d()))),
        new InstantCommand(() -> slide.slideArmUp()),
        new LineToLinearPathCommand(drive, S3Start.toPose2d()),
        slide
            .swipeCommand()
            .alongWith(
                new WaitCommand(pathStartToSwipe)
                    .andThen(new LineToLinearPathCommand(drive, S3End.toPose2d()))));
    //        new LineToLinearPathCommand(drive, grabSpecPose.toPose2d())
    //            .alongWith(switchToSpecimenMode()));
    //        specimenCycle(spec1Pose.toPose2d()),
    //        new WaitCommand(pathToGrab),
    //        specimenCycle(spec2Pose.toPose2d()),
    //        new WaitCommand(pathToGrab),
    //        specimenCycle(spec3Pose.toPose2d()),
    //        new WaitCommand(pathToGrab),
    //        specimenCycle(spec4Pose.toPose2d()),
    //        new WaitCommand(pathToGrab),
    //        specimenCycle(spec5Pose.toPose2d()));
  }

  public Command hangSpecimen() {
    return new InstantCommand(() -> lift.setGoal(Lift.Goal.PRE_HANG));
  }

  public Command stowLiftAndArm() {
    return new SequentialCommandGroup(
        liftClaw.setLiftClawServo(LiftClaw.LiftClawState.GRAB_FROM_WALL, 100),
        new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW)));
  }

  public Command specimenCycle(Pose2d specHangPose) {
    return new SequentialCommandGroup(
        liftClaw.closeClawCommand(),
        new LineToLinearPathCommand(drive, specHangPose)
            .alongWith(new WaitCommand(delayToUpLift).andThen(upLiftToHang())),
        hangSpecimen(),
        new WaitCommand(hangToOpenClaw),
        liftClaw.openClawCommand(),
        new LineToLinearPathCommand(drive, grabSpecPose.toPose2d())
            .alongWith(new WaitCommand(delayToStow).andThen(stowLiftAndArm())));
  }

  // slide Arm 0.22 wrist 0.6

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
