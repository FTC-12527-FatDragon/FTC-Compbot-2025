package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommand.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.SlideSuperStucture;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TrajectoryManager;

@Config
@Autonomous(name = "AutoTest", group = "Autos")
public class AutoTest extends LinearOpMode {
  public static double chamberSpacing = 3;
  public static long handOff2TrajDelay = 400;

  // Chamber hang location
  public static double xValue1 = 16;
  public static double yValue1 = -32.5;
  public static double heading1 = 90;

  public static double takeoffX = xValue1/5;
  public static double takeoffY = yValue1/5;

  // Grab location
  public static double xValue2 = -4.5;
  public static double yValue2 = -6;
  public static double heading2 = -180;

  // The middle sample
  public static double xValue3 = 10;
  public static double yValue3 = 0;
  public static double heading3 = -90;

  // The leftmost sample
  public static double xValue4 = 0;
  public static double yValue4 = 0;
  public static double heading4 = -90;

  // The Ascent zone
  public static double xValue5 = 10;
  public static double yValue5 = 10;
  public static double heading5 = -90;

  LiftClaw liftClaw;
  Lift lift;
  SlideSuperStucture slide;

  Pose2d startPose = new Pose2d(0, 0, Math.toRadians(heading3));

  // Start to Basket
  TrajectorySequence trajs1 =
          TrajectoryManager.trajectorySequenceBuilder(startPose)
                  .turn(90)
                  .build();

  // Basket to the rightmost sample
  TrajectorySequence trajs2 =
          TrajectoryManager.trajectorySequenceBuilder(trajs1.end())
                  .lineToLinearHeading(new Pose2d(xValue2, yValue2, Math.toRadians(heading2)))
                  .build();

  // rightmost sample to basket
  TrajectorySequence trajs3 =
          TrajectoryManager.trajectorySequenceBuilder(trajs2.end())
                  .lineToLinearHeading(
                          new Pose2d(xValue1 - chamberSpacing, yValue1, Math.toRadians(heading1)))
                  .build();

  // basket to middle sample
  TrajectorySequence trajs4 =
          TrajectoryManager.trajectorySequenceBuilder(trajs3.end())
                  .lineToLinearHeading(new Pose2d(xValue2, yValue2, Math.toRadians(heading2)))
                  .build();

  // middle sample to basket
  TrajectorySequence trajs5 =
          TrajectoryManager.trajectorySequenceBuilder(trajs4.end())
                  .lineToLinearHeading(
                          new Pose2d(xValue1 - chamberSpacing * 2, yValue1, Math.toRadians(heading1)))
                  .build();

  // basket to leftmost sample
  TrajectorySequence trajs6 =
          TrajectoryManager.trajectorySequenceBuilder(trajs5.end())
                  .lineToLinearHeading(new Pose2d(xValue2, yValue2, Math.toRadians(heading2)))
                  .build();

  // leftmost sample to basket
  TrajectorySequence trajs7 =
          TrajectoryManager.trajectorySequenceBuilder(trajs6.end())
                  .lineToLinearHeading(
                          new Pose2d(xValue1 - chamberSpacing * 3, yValue1, Math.toRadians(heading1)))
                  .build();

  // basket to ascent zone
  TrajectorySequence trajs8 =
          TrajectoryManager.trajectorySequenceBuilder(trajs7.end())
                  .lineToLinearHeading(new Pose2d(xValue2, yValue2, Math.toRadians(heading2)))
                  .build();

  public Command wait(SampleMecanumDrive drive, long ms) {
    return new ParallelDeadlineGroup(
        new WaitCommand(ms),
        new FunctionalCommand(
            () -> {},
            drive::update,
            (b) -> {},
            () -> {
              return drive.isBusy() && !isStopRequested();
            }));
  }

  @Override
  public void runOpMode() throws InterruptedException {
    this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    CommandScheduler.getInstance().reset();

    // Subsystems Initialized
    lift = new Lift(hardwareMap, telemetry);
    liftClaw = new LiftClaw(hardwareMap);
    slide = new SlideSuperStucture(hardwareMap, telemetry);

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    drive.setPoseEstimate(new Pose2d(xValue5,yValue5,heading5));

    // Score the first chamber
    // Push all three samples to the observation zone
    // Repeatedly score the high chamber with slightly different
    slide.stow();
    slide.backwardSlideExtension();
    liftClaw.closeClaw();
    liftClaw.foldLiftArm();

    waitForStart();

    // spotless:off

    CommandScheduler.getInstance()
            .schedule(
                    new SequentialCommandGroup(
                            followTrajectory(drive, trajs1)
                    ));
    //spotless:on

    while (opModeIsActive() && !isStopRequested()) {
      CommandScheduler.getInstance().run();
      lift.periodicTest();
    }
  }
}
