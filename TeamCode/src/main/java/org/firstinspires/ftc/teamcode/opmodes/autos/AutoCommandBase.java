package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.concurrent.atomic.AtomicReference;
import lombok.Getter;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.lib.Units;
import org.firstinspires.ftc.teamcode.lib.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.SlideSuperStucture;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TrajectoryManager;
import org.firstinspires.ftc.teamcode.utils.Pose2dHelperClass;

/**
 * Layout: Field Coordinate: +-----+-----+-----+-----+-----+-----^ x | 0 | 1 | 2 | 3 | 4 | 5 |
 * <-----+-----+-----+-----+-----+-----+ CCW+ y O Robot Coordinate:
 *
 * <p>Center
 */
@Config
public abstract class AutoCommandBase extends LinearOpMode {
  protected LiftClaw liftClaw;
  protected Lift lift;
  protected SlideSuperStucture slide;
  protected SampleMecanumDrive drive;
  protected Climber climb;
  protected Vision vision;

  public static long handoff_slide2LiftCloseDelayMs = 150;
  public static long handoff_liftClose2OpenIntakeDelayMs = 50;

  public static class FieldConfig {
    public double blockSideLengthInch = 24;
  }

  private static TrajectorySequence sequence = null;

  public static FieldConfig Field = new FieldConfig();

  public static class RobotConfig {
    public double robotBack2FrontXLenInch = 16.5;
    public double robotLeft2RightYLenInch = 15.5;
  }

  public static RobotConfig Robot = new RobotConfig();
  //
  //  public enum StartPoseConfig{
  //    L0(robotCentral, true),
  //    R0(robotCentral, false),
  //    L1(robotCentral, true),
  //    R1(robotCentral, false),
  //    L2(robotCentral, true),
  //    R2(robotCentral, false),
  //    L3(robotCentral, true),
  //    R3(robotCentral, false),
  //    L4(robotCentral, true),
  //    R4(robotCentral, false),
  //    L5(robotCentral, true),
  //    R5(new Vector2d(0, 0), false);
  //    public final Vector2d startPose;
  //    StartPoseConfig(final Vector2d startPose, boolean isLeft){
  //      if(isLeft){
  //        this.startPose = new Vector2d(startPose.getX(), startPose.getY() - robotCentral.getY());
  //      }else{
  //        this.startPose = startPose.plus(robotCentral);
  //      }
  //    }
  //  }
  //  public static StartPoseConfig StartPose = StartPoseConfig.L1;
  //
  //  public enum StartHeadingConfig{
  //    LEFT(Math.toRadians(-90)),
  //    FRONT(Math.toRadians(0)),
  //    RIGHT(Math.toRadians(90)),
  //    BACK(Math.toRadians(180));
  //    public final double heading;
  //    StartHeadingConfig(double headingrad){
  //      heading = headingrad;
  //    }
  //  }
  //  public static StartHeadingConfig StartHeading = StartHeadingConfig.FRONT;
  //  protected static Vector2d robotCentral = new Vector2d(Robot.robotBack2FrontXLenInch,
  // Robot.robotLeft2RightYLenInch);
  //  protected static Pose2d startPose = new Pose2d(StartPose.startPose, StartHeading.heading);
  @Getter private static Pose2d autoEndPose = new Pose2d();

  protected void initialize() {
    initialize(true);
  }

  protected void initialize(boolean telemetryInDashboard) {
    if (telemetryInDashboard) {
      this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    CommandScheduler.getInstance().reset();
    // Subsystems Initialized
    lift = new Lift(hardwareMap, telemetry);
    liftClaw = new LiftClaw(hardwareMap);
    climb = new Climber(hardwareMap);
    slide = new SlideSuperStucture(hardwareMap, telemetry);
    drive = new SampleMecanumDrive(hardwareMap);
    vision = new Vision(hardwareMap, telemetry);

    slide.stow();
    slide.backwardSlideExtension();
    liftClaw.closeClaw();
    liftClaw.foldLiftArm();
    vision.initializeCamera();
    //    drive.setPoseEstimate(startPose);
  }

  protected Command upLiftToBasket() {
    return new ParallelCommandGroup(
        new InstantCommand(() -> lift.setGoal(Lift.Goal.HIGH_BASKET)),
        new WaitUntilCommand(() -> lift.getCurrentPosition() > 300)
            .andThen(liftClaw.setLiftClawServo(LiftClaw.LiftClawState.SCORE_BASKET, 0)));
  }

  protected Command followTrajectory(TrajectorySequence trajectorySequence) {
    return new AutoDriveCommand(drive, trajectorySequence);
  }

  protected Command stowArmFromBasket() {
    return new SequentialCommandGroup(
        liftClaw.openClawCommand(),
        new WaitCommand(100),
        liftClaw.setLiftClawServo(LiftClaw.LiftClawState.STOW, 200),
        new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW)));
  }

  protected Command slowHandoff() {
    return slowHandoff(slide, liftClaw);
  }

  public static Command slowHandoff(SlideSuperStucture slide, LiftClaw liftClaw) {
    return slide
        .slowHandoffCommand()
        .beforeStarting(liftClaw::openClaw)
        .andThen(new WaitCommand(handoff_slide2LiftCloseDelayMs))
        .andThen(liftClaw.closeClawCommand())
        .andThen(new WaitCommand(handoff_liftClose2OpenIntakeDelayMs))
        .andThen(new InstantCommand(slide::openIntakeClaw));
  }

  protected Command fastHandoff() {
    return fastHandoff(slide, liftClaw);
  }

  public static Command fastHandoff(SlideSuperStucture slide, LiftClaw liftClaw) {
    return new SequentialCommandGroup(
        liftClaw.openClawCommand(),
        slide.fastHandoffCommand().andThen(new WaitCommand(handoff_slide2LiftCloseDelayMs)),
        liftClaw.closeClawCommand(),
        new WaitCommand(handoff_liftClose2OpenIntakeDelayMs),
        new InstantCommand(slide::openIntakeClaw));
  }

  protected Command upLiftToChamber() {
    return new ParallelCommandGroup(
        new InstantCommand(() -> lift.setGoal(Lift.Goal.PRE_HANG)),
        new WaitUntilCommand(() -> lift.getCurrentPosition() > 90)
            .andThen(liftClaw.setLiftClawServo(LiftClaw.LiftClawState.SCORE_BASKET, 0)));
  }

  protected Command handoffAndLiftToChamber() {
    return fastHandoff()
        .andThen(new WaitCommand(150))
        .andThen(new InstantCommand(slide::slideArmDown))
        .andThen(new WaitCommand(150))
        .andThen(upLiftToChamber());
  }

  protected Command hangAndStowLift() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> lift.setGoal(Lift.Goal.HANG))
            .alongWith(new InstantCommand(slide::slideArmDown)),
        new ParallelDeadlineGroup(new WaitCommand(100), new WaitUntilCommand(() -> lift.atHome(10)))
            .andThen(
                new WaitCommand(160)
                    .deadlineWith(
                        lift.manualResetCommand()
                            .alongWith(
                                new SequentialCommandGroup(
                                    new WaitCommand(100),
                                    new InstantCommand(
                                        () -> drive.setWeightedDrivePower(new Pose2d(1, 0, 0))),
                                    new WaitCommand(50)))))
            .andThen(
                new InstantCommand(
                    () -> {
                      drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
                      liftClaw.openClaw();
                    })),
        new WaitCommand(50),
        liftClaw.setLiftClawServo(LiftClaw.LiftClawState.STOW, 0),
        new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW)));
  }

  public Command wait(SampleMecanumDrive drive, long ms) {
    return new ParallelDeadlineGroup(
        new WaitCommand(ms), new RunCommand(drive::update).interruptOn(this::isStopRequested));
  }

  public Command initializeCommand() {
    return new ParallelCommandGroup(
        //        new InstantCommand(slide::forwardSlideExtension),
        liftClaw.closeClawCommand(),
        new InstantCommand(slide::slideArmUp),
        new InstantCommand(slide::wristUp),
        new InstantCommand(slide::openIntakeClaw));
  }

  public Command autoFinish() {
    return new ParallelCommandGroup(
        slide.aimCommand(),
        // TODO: needs discussion
        slide.manualResetCommand().withTimeout(1000), // interruptOn(slide::atHome),
        // lift.resetCommand().interruptOn(() -> lift.atHome(3)),
        lift.manualResetCommand().withTimeout(1000),
        liftClaw.openClawCommand(),
        new InstantCommand(() -> autoEndPose = drive.getPoseEstimate()));
  }

  public static Command generateVisionPath(
      AtomicReference<TrajectorySequence> sequenceContainer,
      AtomicReference<Pose2d> currentPose,
      AtomicReference<Pose2d> targetPose) {
    Runnable pathGeneration =
        () -> {
          Pose2d currentPoseRelativeToField = currentPose.get();
          Pose2d targetPoseRelativeToRobot = targetPose.get();
          // Transform target pose from robot-relative to field-relative coordinates
          double fieldX =
              currentPoseRelativeToField.getX()
                  + (targetPoseRelativeToRobot.getX()
                          * Math.cos(currentPoseRelativeToField.getHeading())
                      - targetPoseRelativeToRobot.getY()
                          * Math.sin(currentPoseRelativeToField.getHeading()));
          double fieldY =
              currentPoseRelativeToField.getY()
                  + (targetPoseRelativeToRobot.getX()
                          * Math.sin(currentPoseRelativeToField.getHeading())
                      + targetPoseRelativeToRobot.getY()
                          * Math.cos(currentPoseRelativeToField.getHeading()));
          double fieldHeading =
              currentPoseRelativeToField.getHeading()
                  + 0; // We move our claw instead of robot heading

          Pose2d targetPoseRelativeToField = new Pose2d(fieldX, fieldY, fieldHeading);

          // Generate trajectory to target pose
          sequenceContainer.set(
              TrajectoryManager.trajectorySequenceBuilder(currentPoseRelativeToField)
                  .lineToLinearHeading(targetPoseRelativeToField)
                  .build());
        };
    return new InstantCommand(pathGeneration);
  }

  public static Command alignToSample(
      SampleMecanumDrive drive, Vision vision, Telemetry telemetry) {
    AtomicReference<TrajectorySequence> sequenceContainer = new AtomicReference<>();
    //    AtomicReference<Pose2d> currentPoseRelativeToField = new AtomicReference<>();
    //    AtomicReference<Pose2d> targetPoseRelativeToRobot = new AtomicReference<>();

    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              Pose2d currentPoseRelativeToField = drive.getPoseEstimate();
              telemetry.addData("Current Pose", currentPoseRelativeToField);
              Pose2d targetPoseRelativeToRobot =
                  new Pose2dHelperClass(
                          vision.getDistance() / 25.4,
                          vision.getStrafeOffset() / 25.4,
                          0)
                      .toPose2d();
              telemetry.addData("Target Robot Pose", targetPoseRelativeToRobot);

              //              Pose2d currentPoseRelativeToField = currentPose.get();
              //              Pose2d targetPoseRelativeToRobot = targetPose.get();
              // Transform target pose from robot-relative to field-relative coordinates
              double fieldX =
                  currentPoseRelativeToField.getX()
                      + (targetPoseRelativeToRobot.getX()
                              * Math.cos(currentPoseRelativeToField.getHeading())
                          - targetPoseRelativeToRobot.getY()
                              * Math.sin(currentPoseRelativeToField.getHeading()));
              double fieldY =
                  currentPoseRelativeToField.getY()
                      + (targetPoseRelativeToRobot.getX()
                              * Math.sin(currentPoseRelativeToField.getHeading())
                          + targetPoseRelativeToRobot.getY()
                              * Math.cos(currentPoseRelativeToField.getHeading()));
              double fieldHeading =
                  currentPoseRelativeToField.getHeading()
                      + 0; // We move our claw instead of robot heading

              Pose2d targetPoseRelativeToField = new Pose2d(fieldX, fieldY, fieldHeading);
              telemetry.addData("Target Field Pose", targetPoseRelativeToField);

              // Generate trajectory to target pose
              sequence =
                  TrajectoryManager.trajectorySequenceBuilder(currentPoseRelativeToField)
                      .lineToLinearHeading(targetPoseRelativeToField)
                      .build();


              CommandScheduler.getInstance().schedule(new AutoDriveCommand(drive, sequence));
            }));
  }

  /**
   * Gets the command to run in auto, this should be implemented in each auto.
   *
   * @return The command to run.
   */
  public abstract Command runAutoCommand();

  /**
   * Gets the robot starting pose in field coordinate or its respective coordinates.
   *
   * @return The start pose following RoadRunner's coordinate system.
   */
  public abstract Pose2d getStartPose();

  @Override
  public void runOpMode() throws InterruptedException {
    double origval = SlideSuperStucture.IntakeClawServo_OPEN;
    initialize();
    SlideSuperStucture.IntakeClawServo_OPEN = SlideSuperStucture.IntakeClawServo_OPENWIDER;
    drive.setPoseEstimate(getStartPose());
    Command toRun = runAutoCommand().andThen(autoFinish());
    waitForStart();

    CommandScheduler.getInstance().schedule(toRun);

    while (opModeIsActive() && !isStopRequested()) {
      lift.periodicAsync();
      CommandScheduler.getInstance().run();
    }
    SlideSuperStucture.IntakeClawServo_OPEN = origval;
  }
}
