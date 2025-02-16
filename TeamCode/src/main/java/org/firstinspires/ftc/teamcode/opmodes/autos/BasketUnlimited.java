package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.commands.LineToLinearPathCommand;
import org.firstinspires.ftc.teamcode.commands.SplineToPathCommand;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Pose2dHelperClass;

@Config
@Autonomous(name = "Basket ∞", group = "Autos")
public class BasketUnlimited extends AutoCommandBase {

  // For Basket Scoring
  public static Pose2dHelperClass Basket = new Pose2dHelperClass(-56, -56, 45);

  public static Pose2dHelperClass BasketForSpline = new Pose2dHelperClass(-59, -54.5, 60);

  public static Pose2dHelperClass S1Basket =
      new Pose2dHelperClass(-60.124, -54.577, Math.toDegrees(1.1506));

  public static Pose2dHelperClass S2Basket =
      new Pose2dHelperClass(-64.694, -54.293, Math.toDegrees(1.621));

  // The right sample
  public static Pose2dHelperClass S1 =
      new Pose2dHelperClass(-59.8, -48.99884, Math.toDegrees(1.12748));

  // The middle sample
  public static Pose2dHelperClass S2 =
      new Pose2dHelperClass(-62.8178, -50.8168, Math.toDegrees(1.4281));

  // The left sample
  public static Pose2dHelperClass S3 =
      new Pose2dHelperClass(-63.25, -48.5, 113);

  // Middle point for spline
  public static Pose2dHelperClass splinePoint1 = new Pose2dHelperClass(-24.44, -10, 0);

  public static Pose2dHelperClass splinePoint2 = new Pose2dHelperClass(-24.44, -8, 0);

  public static long basketWaitMs = 650;

  public static long basketWaitForAutoPickMs = 180;

  public static long firstBasketWaitMs = 500;

  public static long pick2Handoff = 0;

  public static long startStowToPath = 0;

  public static long stowedToGrabDelay = 0;

  public static long stopLinearToLLM = 0;

  public static long S3SlideDelay = 0;

  public static double S1TurnPos = 0.35;
  public static double S2TurnPos = 0.25;
  public static double S3TurnPos = 0.17;

  public static Pose2d startPose = new Pose2d(-40.13, -63.82, Math.toRadians(90));

  @Override
  public Pose2d getStartPose() {
    return drive.getPoseEstimate();
  }

  @Override
  public void initializeSuperStructure() {
    drive.breakFollowing(true);
    slide.stow();
    slide.openIntakeClaw();
    slide.backwardSlideExtension();
    liftClaw.closeClaw();
    liftClaw.foldLiftArm();
    vision.initializeCamera();
    vision.setLEDPWM();
  }

  // spotless:off
  @Override
  public Command runAutoCommand() {
    Supplier<Command> slideExtendCommand =
        () -> new InstantCommand(() -> slide.forwardSlideExtension());

    return new SequentialCommandGroup(
        new InstantCommand(
                () -> {
                  drive.setPoseEstimate(startPose);
                  drive.setCurrentTrajectoryMode(SampleMecanumDrive.TrajectoryMode.SLOW);
                })
            .beforeStarting(liftClaw::closeClaw),
        new LineToLinearPathCommand(drive, S1Basket.toPose2d(), false)
            .alongWith(
                slide
                    .aimCommand()
                    .alongWith(new InstantCommand(() -> slide.setTurnServo(S1TurnPos))),
                upLiftToBasket()
                    .andThen(
                        wait(drive, firstBasketWaitMs),
                        stowArmFromBasket().alongWith(slideExtendCommand.get()))),
        new LineToLinearPathCommand(drive, S1.toPose2d())
            .alongWith(
                new WaitUntilCommand(() -> lift.atHome(15))
                    .andThen(new WaitCommand(stowedToGrabDelay), slide.grabCommand())),
        new LineToLinearPathCommand(drive, S1Basket.toPose2d())
            .alongWith(fastHandoff().andThen(upLiftToBasket())),
        wait(drive, basketWaitMs),
        stowArmFromBasket()
            .alongWith(
                slide
                    .aimCommand()
                    .alongWith(new InstantCommand(() -> slide.setTurnServo(S2TurnPos))),
                slideExtendCommand.get()),
        new LineToLinearPathCommand(drive, S2.toPose2d())
            .alongWith(
                new WaitUntilCommand(() -> lift.atHome(15))
                    .andThen(new WaitCommand(stowedToGrabDelay), slide.grabCommand())),
        new LineToLinearPathCommand(drive, S2Basket.toPose2d())
            .alongWith(fastHandoff().andThen(upLiftToBasket())),
        wait(drive, basketWaitMs),
        stowArmFromBasket()
            .alongWith(
                slide
                    .aimCommand()
                    .alongWith(new InstantCommand(() -> slide.setTurnServo(S3TurnPos))),
                new WaitCommand(S3SlideDelay).andThen(slideExtendCommand.get())),
        new LineToLinearPathCommand(drive, S3.toPose2d())
            .alongWith(
                new WaitUntilCommand(() -> lift.atHome(15))
                    .andThen(new WaitCommand(stowedToGrabDelay), slide.grabCommand())),
        new LineToLinearPathCommand(drive, BasketForSpline.toPose2d())
            .alongWith(fastHandoff().andThen(upLiftToBasket())),
        wait(drive, basketWaitMs),
        stowArmFromBasket()
            .alongWith(
                slowHandoff(),
                new WaitCommand(startStowToPath)
                    .andThen(
                        new SplineToPathCommand(drive, splinePoint1.toPose2d(), 0)
                            .andThen(new WaitCommand(stopLinearToLLM)))),

        // new SplineToPathCommand(drive, splinePoint1.toPose2d(), goPickTangent),
        //        wait(drive, 100),
        autoSamplePickCommand(),
        new SplineToPathCommand(drive, BasketForSpline.toPose2d(), true)
            .alongWith(new WaitCommand(pick2Handoff).andThen(slowHandoff(), upLiftToBasket())),
        wait(drive, basketWaitForAutoPickMs),
        stowArmFromBasket()
            .alongWith(
                slowHandoff(),
                new WaitCommand(startStowToPath)
                    .andThen(
                        new SplineToPathCommand(drive, splinePoint2.toPose2d(), 0)
                            .andThen(new WaitCommand(stopLinearToLLM)))),
        // new SplineToPathCommand(drive, splinePoint2.toPose2d(), goPickTangent),
        //        wait(drive, 100),
        autoSamplePickCommand(),
        new SplineToPathCommand(drive, BasketForSpline.toPose2d(), true)
            .alongWith(new WaitCommand(pick2Handoff).andThen(slowHandoff(), upLiftToBasket())),
        wait(drive, basketWaitForAutoPickMs),
        stowArmFromBasket()
            .alongWith(
                slowHandoff(),
                new WaitCommand(startStowToPath)
                    .andThen(
                        new SplineToPathCommand(drive, splinePoint1.toPose2d(), 0)
                            .andThen(new WaitCommand(stopLinearToLLM)))),
        // new SplineToPathCommand(drive, splinePoint1.toPose2d(), goPickTangent),
        //        wait(drive, 100),
        autoSamplePickCommand(),
        new SplineToPathCommand(drive, BasketForSpline.toPose2d(), true)
            .alongWith(new WaitCommand(pick2Handoff).andThen(slowHandoff(), upLiftToBasket())),
        wait(drive, basketWaitForAutoPickMs),
        stowArmFromBasket().alongWith(slowHandoff()));
  }
  // spotless:
}
