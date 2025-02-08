package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import edu.wpi.first.math.MathUtil;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.lib.roadrunner.drive.opmode.LocalizationTest;
import org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystems.SlideSuperStucture;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;
import org.firstinspires.ftc.teamcode.utils.Pose2dHelperClass;

@Config
@TeleOp(name = "Solo")
public class TXBetaBotSolo extends CommandOpMode {
  private GamepadEx gamepadEx1;
  private GamepadEx gamepadEx2;
  private Lift lift;
  private Climber climber;
  private LiftClaw liftClaw;
  private SlideSuperStucture slide;
  private SampleMecanumDrive drive;
  private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

  private DriverMode currentMode = DriverMode.SAMPLE;

  public static boolean setPose = false;
  public static Pose2dHelperClass Pose = new Pose2dHelperClass();
  private boolean isTimerStart = false;
  private boolean lowBasketMode = false;

  @Override
  public void initialize() {
    this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    gamepadEx1 = new GamepadEx(gamepad1);
    gamepadEx2 = new GamepadEx(gamepad2);

    lift = new Lift(hardwareMap, telemetry);
    climber = new Climber(hardwareMap);
    liftClaw = new LiftClaw(hardwareMap);
    slide = new SlideSuperStucture(hardwareMap, telemetry);
    drive = new SampleMecanumDrive(hardwareMap);

    initializeMode();

    isTimerStart = false;

    // climber = new Climber(hardwareMap);
    drive.setPoseEstimate(AutoCommandBase.getAutoEndPose());

    // Teleop Drive Command
    drive.setDefaultCommand(
        new TeleopDriveCommand(
            drive,
            () -> -gamepadEx1.getLeftY(),
            () -> gamepadEx1.getLeftX(),
            () -> gamepadEx1.getRightX(),
            () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON),
            () -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT)));

    // =================================================================================

    // SAMPLE MODE

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.START) && currentMode == DriverMode.SAMPLE)
        .whenPressed(() -> lowBasketMode = !lowBasketMode);



    // BASKET UP
    new FunctionalButton(
            () -> gamepadEx1.getButton(GamepadKeys.Button.X) && currentMode == DriverMode.SAMPLE)
        .whenPressed(
            new ParallelCommandGroup(
                new ConditionalCommand(
                    new InstantCommand(() -> lift.setGoal(Lift.Goal.HIGH_BASKET)),
                    new InstantCommand(() -> lift.setGoal(Lift.Goal.LOW_BASKET)),
                    () -> !lowBasketMode),
                new WaitUntilCommand(() -> lift.getCurrentPosition() > 150)
                    .andThen(liftClaw.setLiftClawServo(LiftClaw.LiftClawState.SCORE_BASKET, 0))));

    // BASKET DROP OFF AND STOW
    // IF HANG ALSO --> STOW AND STOW
    new FunctionalButton(
            () -> gamepadEx1.getButton(GamepadKeys.Button.B) && currentMode == DriverMode.SAMPLE)
        .whenPressed(
            liftClaw
                .openClawCommand()
                .andThen(
                    new WaitCommand(100),
                    liftClaw.setLiftClawServo(LiftClaw.LiftClawState.STOW, 200),
                    new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW))));

    // FIRST AIM
    new FunctionalButton(
            () -> gamepadEx1.getButton(GamepadKeys.Button.Y) && currentMode == DriverMode.SAMPLE)
        .whenPressed(slide.aimCommand(), false);

    // Grab when aim
    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.A)
                    && slide.getGoal() == SlideSuperStucture.Goal.AIM
                    && currentMode == DriverMode.SAMPLE)
        .whenPressed(slide.grabCommand(), false);

    // Pure Handoff
    Supplier<Command> slowHandoffSCommand =
        () -> AutoCommandBase.slowHandoff(slide, liftClaw).andThen(new WaitCommand(50));

    Supplier<Command> fastHandoffCommand = // TODO: Remove duplicate code
        () -> AutoCommandBase.fastHandoff(slide, liftClaw);

    Supplier<Command> handoffCommand =
        () ->
            new ConditionalCommand(
                fastHandoffCommand.get(), slowHandoffSCommand.get(), slide::isSlideExtended);

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT)
                    && slide.getGoal() == SlideSuperStucture.Goal.AIM
                    && lift.getGoal() == Lift.Goal.STOW
                    && currentMode == DriverMode.SAMPLE)
        .whenPressed(handoffCommand.get());

    new FunctionalButton(
            () ->
                gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5
                    && slide.getGoal() == SlideSuperStucture.Goal.AIM
                    && currentMode == DriverMode.SAMPLE)
        .whenPressed(new SequentialCommandGroup(new InstantCommand(slide::backwardSlideExtension)));

    new FunctionalButton(
            () ->
                gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5
                    && slide.getGoal() == SlideSuperStucture.Goal.AIM
                    && currentMode == DriverMode.SAMPLE)
        .whenPressed(new InstantCommand(slide::forwardSlideExtension));

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER)
                    && slide.getGoal() == SlideSuperStucture.Goal.AIM
                    && currentMode == DriverMode.SAMPLE)
        .whenPressed(new InstantCommand(() -> slide.leftTurnServo()));

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                    && slide.getGoal() == SlideSuperStucture.Goal.AIM
                    && currentMode == DriverMode.SAMPLE)
        .whenPressed(new InstantCommand(() -> slide.rightTurnServo()));

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN)
                    && lift.atHome(10)
                    && currentMode == DriverMode.SPECIMEN)
        .whenPressed(
            new SequentialCommandGroup(
                new InstantCommand(() -> currentMode = DriverMode.SAMPLE),
                liftClaw.setLiftClawServo(LiftClaw.LiftClawState.AVOID_COLLISION, 200),
                slide.aimCommand(),
                new WaitCommand(200),
                liftClaw.setLiftClawServo(LiftClaw.LiftClawState.STOW, 0),
                liftClaw.openClawCommand()));

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN)
                    && currentMode == DriverMode.CLIMB)
        .whenPressed(new InstantCommand(() -> currentMode = DriverMode.SAMPLE));

    // =================================================================================

    // SPECIMEN MODE
    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)
                    && (slide.getGoal() == SlideSuperStucture.Goal.AIM
                        || slide.getGoal() == SlideSuperStucture.Goal.HANDOFF)
                    && slide.slideMotorAtHome()
                    && currentMode == DriverMode.SAMPLE)
        .whenPressed(
            new SequentialCommandGroup(
                new ConditionalCommand(
                    new InstantCommand(slide::slideArmUp).andThen(new WaitCommand(100)),
                    new InstantCommand(),
                    () -> slide.getGoal() == SlideSuperStucture.Goal.HANDOFF),
                new InstantCommand(() -> currentMode = DriverMode.SPECIMEN),
                liftClaw.setLiftClawServo(LiftClaw.LiftClawState.AVOID_COLLISION, 200),
                slide.foldSlideStructureCommand(),
                new WaitCommand(200),
                liftClaw.setLiftClawServo(LiftClaw.LiftClawState.GRAB_FROM_WALL, 0),
                liftClaw.openClawCommand()));

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP) && currentMode == DriverMode.CLIMB)
        .whenPressed(new InstantCommand(() -> currentMode = DriverMode.SPECIMEN));

    // Right Bumper Claw Open
    // Right Trigger Up Down
    // Left Bumper Flip

    new FunctionalButton(
            () -> gamepadEx1.getButton(GamepadKeys.Button.B) && currentMode == DriverMode.SPECIMEN)
        .whenPressed(
            new SequentialCommandGroup(
                liftClaw.openClawCommand(),
                new WaitCommand(100),
                liftClaw.setLiftClawServo(LiftClaw.LiftClawState.GRAB_FROM_WALL, 100),
                new InstantCommand(() -> lift.setGoal(Lift.Goal.STOW))),
            false);

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                    && currentMode == DriverMode.SPECIMEN)
        .whenPressed(liftClaw.switchClawCommand(), false);

    new FunctionalButton(
            () ->
                gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5
                    && lift.getGoal() == Lift.Goal.STOW
                    && currentMode == DriverMode.SPECIMEN)
        .whenPressed(
            new SequentialCommandGroup(
                liftClaw.setLiftClawServo(LiftClaw.LiftClawState.SCORE_CHAMBER, 200),
                new InstantCommand(() -> lift.setGoal(Lift.Goal.HANG))));

    new FunctionalButton(
            () ->
                gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5
                    && lift.getGoal() == Lift.Goal.HANG
                    && currentMode == DriverMode.SPECIMEN)
        .whenPressed(new InstantCommand(() -> lift.setGoal(Lift.Goal.PRE_HANG)));

    new FunctionalButton(
            () ->
                gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5
                    && lift.getGoal() == Lift.Goal.PRE_HANG
                    && currentMode == DriverMode.SPECIMEN)
        .whenPressed(new InstantCommand(() -> lift.setGoal(Lift.Goal.HANG)));

    // =================================================================================

    // Manual Reset
    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                    && lift.getGoal() == Lift.Goal.STOW)
        .whenHeld(lift.manualResetCommand());

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                    && slide.getGoal().slideExtension == 0)
        .whenHeld(slide.manualResetCommand());

    // =================================================================================

    // Climb Mode
    new FunctionalButton(() -> gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT))
        .whenPressed(new InstantCommand(() -> currentMode = DriverMode.CLIMB));

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                    && currentMode == DriverMode.CLIMB)
        .whenHeld(climber.elevateCommand());

    new FunctionalButton(
            () ->
                gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER)
                    && currentMode == DriverMode.CLIMB)
        .whenHeld(climber.declineCommand());

    new FunctionalButton(
            () -> gamepadEx1.getButton(GamepadKeys.Button.B) && currentMode == DriverMode.CLIMB)
        .toggleWhenPressed(climber.holdOnCommand());

    new FunctionalButton(() -> MathUtil.isNear(110, timer.time(), 0.3))
        .whenPressed(climber.elevateCommand().withTimeout(2000));

    // =================================================================================

    new FunctionalButton(() -> MathUtil.isNear(60, timer.time(), 0.3))
        .whenPressed(new InstantCommand(() -> gamepad1.rumble(500)));

    new FunctionalButton(() -> MathUtil.isNear(90, timer.time(), 0.3))
        .whenPressed(new InstantCommand(() -> gamepad1.rumble(500)));
  }

  @Override
  public void run() {
    if (!isTimerStart) {
      timer.reset();
      isTimerStart = true;
    }

    Pose2d poseEstimate = drive.getPoseEstimate();
    lift.periodicAsync();

    CommandScheduler.getInstance().run();

    TelemetryPacket packet = new TelemetryPacket();

    packet.fieldOverlay().setStroke("#3F51B5");
    LocalizationTest.drawRobot(packet.fieldOverlay(), poseEstimate);
    FtcDashboard.getInstance().sendTelemetryPacket(packet);
    telemetry.addData("X", poseEstimate.getX());
    telemetry.addData("Y", poseEstimate.getY());
    telemetry.addData("Heading", poseEstimate.getHeading());
    if (setPose) {
      setPose = false;
      drive.setPoseEstimate(Pose.toPose2d());
    }
  }

  public enum DriverMode {
    CLIMB,
    SAMPLE,
    SPECIMEN
  }

  @Override
  public void reset() {
    CommandScheduler.getInstance().reset();
    slide.setServoController(false);
    liftClaw.setServoController(false);
  }

  public void initializeMode() {
    boolean isLiftArmStow = liftClaw.isLiftArmStow();
    boolean isSlideArmFold = slide.isSlideArmFold();
    boolean isLiftArmGrab = liftClaw.isLiftArmGrab();

    if (isLiftArmGrab && isSlideArmFold) {
      currentMode = DriverMode.SPECIMEN;
      return;
    }

    // !Not Grab || !Fold

    if (isLiftArmStow) {
      if (isSlideArmFold) {
        CommandScheduler.getInstance()
            .schedule(
                new SequentialCommandGroup(
                    new InstantCommand(() -> currentMode = DriverMode.SPECIMEN),
                    liftClaw.setLiftClawServo(LiftClaw.LiftClawState.AVOID_COLLISION, 200),
                    slide.foldSlideStructureCommand(),
                    new WaitCommand(200),
                    liftClaw.setLiftClawServo(LiftClaw.LiftClawState.GRAB_FROM_WALL, 0),
                    liftClaw.openClawCommand()));
      } else { // Slide Arm aim
        currentMode = DriverMode.SAMPLE;
      }
    } else {
      if (isSlideArmFold) { // FOLD && STOW
        liftClaw.grabFromWall();
        currentMode = DriverMode.SPECIMEN;
      } else { // FOLD && !STOW
        liftClaw.foldLiftArm();
        currentMode = DriverMode.SAMPLE;
      }
    }
  }
}
