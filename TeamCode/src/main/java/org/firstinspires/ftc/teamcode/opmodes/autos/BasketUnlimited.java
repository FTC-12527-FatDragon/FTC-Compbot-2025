package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.commands.AutoPathCommand;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.SlideSuperStucture;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TrajectoryManager;
import org.firstinspires.ftc.teamcode.utils.Pose2dHelperClass;

@Config
@Autonomous(name = "Basket ∞", group = "Autos")
public class BasketUnlimited extends AutoCommandBase {
  public static boolean isAscent = false;

  // For Basket Scoring
  public static Pose2dHelperClass Basket = new Pose2dHelperClass(-56, -56, 45);

  // The right sample
  public static Pose2dHelperClass S3 = new Pose2dHelperClass(-49, -52, 90);

  // The middle sample
  public static Pose2dHelperClass S2 = new Pose2dHelperClass(-57.6, -53, 90);

  // The left sample
  public static Pose2dHelperClass S1Extend = new Pose2dHelperClass(-61, -53.98, 103);

  // Middle point for spline
  public static Pose2dHelperClass splinePoint = new Pose2dHelperClass(-23.44, 1.25, 0);

  public static long basketWaitMs = 200;

  // Ascent zone
  public static double xValue5 = 60;
  public static double yValue5 = -16;
  public static double heading5 = 90;
  public static double tangent5 = -90;

  // Ascent zone for Pick
  public static double xValue6 = 60;
  public static double yValue6 = -16;
  public static double heading6 = 270;
  public static double tangent6 = 0;

  public static long slideExtend2Grab = 150;

  Pose2d startPose = new Pose2d(-40.13, -63.82, Math.toRadians(90));

  // Start to Basket
  TrajectorySequence L12Basket =
      TrajectoryManager.trajectorySequenceBuilder(startPose)
          .splineToLinearHeading(Basket.toPose2d(), Basket.getHeadingRad())
          .build();

  // Basket to the rightmost sample
  TrajectorySequence Basket2S3 =
      TrajectoryManager.trajectorySequenceBuilder(L12Basket.end())
          .lineToLinearHeading(S3.toPose2d())
          .build();

  // rightmost sample to basket
  TrajectorySequence S32Basket =
      TrajectoryManager.trajectorySequenceBuilder(Basket2S3.end())
          .lineToLinearHeading(Basket.toPose2d())
          .build();

  // basket to middle sample
  TrajectorySequence Basket2S2 =
      TrajectoryManager.trajectorySequenceBuilder(S32Basket.end())
          .lineToLinearHeading(S2.toPose2d())
          .build();

  // middle sample to basket
  TrajectorySequence S22Basket =
      TrajectoryManager.trajectorySequenceBuilder(Basket2S2.end())
          .lineToLinearHeading(Basket.toPose2d())
          .build();

  // basket to leftmost sample
  TrajectorySequence Basket2S1Extend =
      TrajectoryManager.trajectorySequenceBuilder(S22Basket.end())
          .lineToLinearHeading(S1Extend.toPose2d())
          .build();

  // leftmost sample to basket
  TrajectorySequence S1Extend2Basket =
      TrajectoryManager.trajectorySequenceBuilder(Basket2S1Extend.end())
          .lineToSplineHeading(Basket.toPose2d())
          .build();

  //  // basket to ascent zone
  TrajectorySequence Basket2Ascent =
      TrajectoryManager.trajectorySequenceBuilder(S1Extend2Basket.end())
          .splineToLinearHeading(new Pose2d(xValue5, yValue5, Math.toRadians(heading5)), tangent5)
          .build();

  TrajectorySequence Basket2Pick =
      TrajectoryManager.trajectorySequenceBuilder(Basket.toPose2d())
          .splineToLinearHeading(splinePoint.toPose2d(), 0)
          .build();

  public Pose2d getStartPose() {
    return drive.getPoseEstimate(); // TODO: return the field relative pose
  }

  @Override
  public Command runAutoCommand() {
    Supplier<Command> slideExtendCommand =
        () -> new InstantCommand(() -> slide.forwardSlideExtension());

    return new SequentialCommandGroup(
        new InstantCommand(() -> drive.setPoseEstimate(L12Basket.start())),
        //            .alongWith(slide.manualResetCommand().withTimeout(200)),
        slide.aimCommand().beforeStarting(liftClaw::closeClaw),
        followTrajectory(L12Basket).alongWith(upLiftToBasket()),
        wait(drive, 100),
        stowArmFromBasket()
            .alongWith(slide.aimCommand())
            .alongWith(new WaitCommand(500).andThen(followTrajectory(Basket2S3))),
        slideExtendCommand.get(),
        wait(drive, slideExtend2Grab),
        slide.grabCommand(),
        followTrajectory(S32Basket).alongWith(fastHandoff().andThen(upLiftToBasket())),
        wait(drive, basketWaitMs),
        stowArmFromBasket()
            .alongWith(slide.aimCommand())
            .alongWith(new WaitCommand(500).andThen(followTrajectory(Basket2S2))),
        slideExtendCommand.get(),
        wait(drive, slideExtend2Grab),
        slide.grabCommand(),
        followTrajectory(S22Basket).alongWith(fastHandoff().andThen(upLiftToBasket())),
        wait(drive, basketWaitMs),
        stowArmFromBasket()
            .alongWith(slide.aimCommand())
            .alongWith(new WaitCommand(500).andThen(followTrajectory(Basket2S1Extend))),
        new InstantCommand(
            () -> {
              slide.forwardSlideExtension(440);
              slide.setServoPos(SlideSuperStucture.TurnServo.DEG_0);
            }),
        wait(drive, slideExtend2Grab),
        slide.grabCommand(),
        fastHandoff(),
        followTrajectory(S1Extend2Basket),
        upLiftToBasket(),
        wait(drive, basketWaitMs),
        stowArmFromBasket(),
        followTrajectory(Basket2Pick),
        new WaitCommand(slideExtend2Grab),
        autoSamplePickCommand(),
        new AutoPathCommand(drive, Basket.toPose2d())
            .alongWith(
                new SequentialCommandGroup(
                    new WaitCommand(300), slowHandoff().andThen(upLiftToBasket()))),
        wait(drive, basketWaitMs),
        stowArmFromBasket(),
        followTrajectory(Basket2Pick),
        new WaitCommand(slideExtend2Grab),
        autoSamplePickCommand(),
        new AutoPathCommand(drive, Basket.toPose2d())
            .alongWith(
                new SequentialCommandGroup(
                    new WaitCommand(300), slowHandoff().andThen(upLiftToBasket()))),
        // wait(drive, basketWaitMs),
        stowArmFromBasket());
  }
}
