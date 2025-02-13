package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.commands.LineToLinearPathCommand;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
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
    return drive.getPoseEstimate();
  }

  @Override
  public void initializeSuperStructure() {
    drive.breakFollowing(true);
    slide.stow();
    slide.backwardSlideExtension();
    liftClaw.foldLiftArm();
    liftClaw.closeClaw();
  }

  private Command grabCycle(Command followCommand, Command nxt){
    return slide.grabCommand().andThen(
            followCommand
                    .alongWith(
                            fastHandoff()
                                    .andThen(
                                            upLiftArmOpen.get()
                                                    .alongWith(slide.aimCommand(), slideExtendCommand.get())
                                    )
                    ).andThen(
                            liftClaw.setLiftClawServo(LiftClaw.LiftClawState.STOW, 0).alongWith(
                                    nxt
                            )
                    )
    );
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

        new LineToLinearPathCommand(drive, S3.toPose2d())
            .alongWith(
                    slideExtendCommand.get(),
                    slide.aimCommand().beforeStarting(() -> slide.setTurnServo(0.35))
            ),

        slide.grabCommand(),
        fastHandoff(),
        new LineToLinearPathCommand(drive, S2.toPose2d()).alongWith(
            upLiftArmOpen.get()
            .alongWith(slide.aimCommand(), slideExtendCommand.get())
        ),
        liftClaw.setLiftClawServo(LiftClaw.LiftClawState.STOW, 200)
                .alongWith(
                        slide.grabCommand()
                )
//        new LineToLinearPathCommand(drive, S1.toPose2d())
//            .alongWith(
//                fastHandoff()
//                    .andThen(
//                        upLiftArmOpen.get()
//                        .alongWith(slide.aimCommand(), slideExtendCommand.get()))
//                    .andThen(
//                        liftClaw.setLiftClawServo(LiftClaw.LiftClawState.STOW, 0)).alongWith(
//                                slide.grabCommand()
//                        )),
//        fastHandoff(),
//        upLiftArmOpen
//            .get()
//            .alongWith(slide.aimCommand())
//            .andThen(
//                new LineToLinearPathCommand(drive, grabSpecPose.toPose2d())
//                    .alongWith(
//                        new WaitCommand(400)
//                            .andThen(
//                                slide.foldSlideStructureCommand(),
//                                new WaitCommand(200),
//                                liftClaw.setLiftClawServo(
//                                    LiftClaw.LiftClawState.GRAB_FROM_WALL, 0)))),
//        liftClaw.closeClawCommand(),
//        new LineToLinearPathCommand(drive, spec1Pose.toPose2d())
//            .alongWith(new WaitCommand(delayToUpLift).andThen(upLiftToHang())),
//        hangSpecimen.get(),
//        wait(drive, hangToOpenClaw),
//        liftClaw.openClawCommand(),
//        new LineToLinearPathCommand(drive, grabSpecPose.toPose2d())
//            .alongWith(new WaitCommand(delayToUpLift).andThen(stowLiftAndArm.get())),
//        liftClaw.closeClawCommand(),
//        new LineToLinearPathCommand(drive, spec2Pose.toPose2d())
//            .alongWith(new WaitCommand(delayToUpLift).andThen(upLiftToHang())),
//        hangSpecimen.get(),
//        wait(drive, hangToOpenClaw),
//        liftClaw.openClawCommand(),
//        new LineToLinearPathCommand(drive, grabSpecPose.toPose2d())
//            .alongWith(new WaitCommand(delayToUpLift).andThen(stowLiftAndArm.get())),
//        liftClaw.closeClawCommand(),
//        new LineToLinearPathCommand(drive, spec3Pose.toPose2d())
//            .alongWith(new WaitCommand(delayToUpLift).andThen(upLiftToHang())),
//        hangSpecimen.get(),
//        wait(drive, hangToOpenClaw),
//        liftClaw.openClawCommand(),
//        new LineToLinearPathCommand(drive, grabSpecPose.toPose2d())
//            .alongWith(new WaitCommand(delayToUpLift).andThen(stowLiftAndArm.get())),
//        liftClaw.closeClawCommand(),
//        new LineToLinearPathCommand(drive, spec4Pose.toPose2d())
//            .alongWith(new WaitCommand(delayToUpLift).andThen(upLiftToHang())),
//        hangSpecimen.get(),
//        wait(drive, hangToOpenClaw),
//        liftClaw.openClawCommand(),
//        new LineToLinearPathCommand(drive, grabSpecPose.toPose2d())
//            .alongWith(new WaitCommand(delayToUpLift).andThen(stowLiftAndArm.get())),
//        liftClaw.closeClawCommand(),
//        new LineToLinearPathCommand(drive, spec5Pose.toPose2d())
//            .alongWith(new WaitCommand(delayToUpLift).andThen(upLiftToHang())),
//        hangSpecimen.get(),
//        wait(drive, hangToOpenClaw),
//        liftClaw.openClawCommand(),
//        new LineToLinearPathCommand(drive, grabSpecPose.toPose2d())
//            .alongWith(stowLiftAndArm.get())
    );
  }
}
