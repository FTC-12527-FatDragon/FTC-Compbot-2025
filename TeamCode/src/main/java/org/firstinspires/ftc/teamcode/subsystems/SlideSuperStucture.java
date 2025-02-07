package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import lombok.Getter;
import lombok.Setter;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

@Config
public class SlideSuperStucture extends MotorPIDSlideSubsystem {
  // ---- Configs ----
  // SlideArmServo
  public static double SlideArmServo_AIM = 0.4;
  public static double SlideArmServo_GRAB = 0.53;
  public static double SlideArmServo_HANDOFF = 0.25;
  public static double SlideArmServo_AIM_ = 0.55;
  public static double SlideArmServo_PREAIM = 0.48;
  public static double SlideArmServo_FOLD = 0.95;

  // intakeClawServo
  public static double IntakeClawServo_OPEN = 0.3;
  public static double IntakeClawServo_OPENWIDER = 0.2;
  public static double IntakeClawServo_GRAB = 0.675;
  // wristServo
  public static double WristServo_UP = 0.05;
  public static double WristServo_DOWN = 0.75;
  public static double WristServo_FOLD = 0.5;

  // wristTurnServo
  public static double WristTurnServo_POS0 = 0.2;
  public static double WristTurnServo_POS1 = 0.6;
  public static double WristTurnServo_POS2 = 0.8;
  // slideMotor
  public static double SlideMotor_atSetPointTolerance = 10;
  public static double SlideMotor_extensionValue = 275;

  // aimCommand
  public static long aimCommand_wristTurn2ArmDelayMs = 0;
  public static long aimCommand_Arm2OpenDelayMs = 20;
  // grabCommand
  public static long grabCommand_armDown2GrabDelayMs = 100;
  public static long grabCommand_grab2AfterGrabDelayMs = 50;
  // slowHandoffCommand
  public static long handoffCommand_wristTurn2wristHandoffDelayMs = 100;
  public static long slowHandoffCommand_wristHandoff2ArmHandoffDelayMs = 250;
  public static long slowHandoffCommand_ArmHandoff2SlideRetractDelayMs = 0;
  // swipeCommand
  public static long swipeCommand_wrist2ExtendDelayMs = 50;

  private final Servo intakeClawServo, wristServo, wristTurnServo;
  private final Servo slideArmServo;
  private final DcMotorEx slideMotor;

  private final PIDController pidController;
  public static double kP = 0.02, kI = 0.0, kD = 0.0005;
  private final VoltageSensor batteryVoltageSensor;

  private static double slideExtensionVal = 0;

  private static double turnAngleDeg = 0.2;
  private TurnServo turnServo = TurnServo.DEG_0;

  @Setter @Getter private Goal goal = Goal.STOW;

  private boolean isFirstAimToggled = false; // For Aim & Pre-Aim Transition

  //  private final Telemetry telemetry; // 0 0.5 0.8

  public static double resetPower = -0.9;

  public SlideSuperStucture(final HardwareMap hardwareMap, final Telemetry telemetry) {
    slideArmServo = hardwareMap.get(Servo.class, "slideArmServo");

    intakeClawServo = hardwareMap.get(Servo.class, "intakeClawServo"); // 0.3 close 0.7 open

    wristServo = hardwareMap.get(Servo.class, "wristServo"); // 0.05 up 0.75 down

    wristTurnServo = hardwareMap.get(Servo.class, "wristTurnServo");

    setServoController(true);

    slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    pidController = new PIDController(kP, kI, kD);
    batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

    this.telemetry = telemetry;
    goal = Goal.STOW;
  }

  public Command setGoalCommand(Goal newGoal) {
    return new InstantCommand(() -> goal = newGoal);
  }

  public Command aimCommand() {
    return new SequentialCommandGroup(
        setGoalCommand(Goal.AIM),
        setTurnServoPosCommand(TurnServo.DEG_0, aimCommand_wristTurn2ArmDelayMs),
        new ConditionalCommand(
            setServoPosCommand(slideArmServo, SlideArmServo_PREAIM, aimCommand_Arm2OpenDelayMs),
            setServoPosCommand(slideArmServo, Goal.AIM.slideArmPos, aimCommand_Arm2OpenDelayMs),
            () -> isFirstAimToggled),
        new InstantCommand(() -> isFirstAimToggled = !isFirstAimToggled),
        new InstantCommand(() -> wristServo.setPosition(Goal.AIM.wristPos)),
        new InstantCommand(() -> intakeClawServo.setPosition(Goal.AIM.clawAngle)));
  }

  //  public Command preAimCommand() {
  //    return new SequentialCommandGroup(
  //        setGoalCommand(Goal.PRE_AIM), setServoPosCommand(slideArmServo, SlideArmServo_PREAIM,
  // 0));
  //  }

  public Command foldSlideStructureCommand() {
    return new SequentialCommandGroup(
        setTurnServoPosCommand(TurnServo.DEG_0, 100),
        setServoPosCommand(slideArmServo, SlideArmServo_FOLD, 100),
        setServoPosCommand(wristServo, WristServo_FOLD, 100),
        new InstantCommand(() -> isFirstAimToggled = false));
  }

  public Command grabCommand() {
    return new SequentialCommandGroup(
        setServoPosCommand(intakeClawServo, Goal.AIM.clawAngle, grabCommand_grab2AfterGrabDelayMs),
        setGoalCommand(Goal.GRAB),
        setServoPosCommand(slideArmServo, Goal.GRAB.slideArmPos, grabCommand_armDown2GrabDelayMs),
        setServoPosCommand(intakeClawServo, Goal.GRAB.clawAngle, grabCommand_grab2AfterGrabDelayMs),
        new InstantCommand(() -> slideArmServo.setPosition(SlideArmServo_AIM_)),
        new InstantCommand(() -> isFirstAimToggled = true),
        setGoalCommand(Goal.AIM));
  }

  public Command slowHandoffCommand() {
    return new SequentialCommandGroup(
        setGoalCommand(Goal.HANDOFF),
        new InstantCommand(() -> isFirstAimToggled = false),
        setTurnServoPosCommand(TurnServo.DEG_0, handoffCommand_wristTurn2wristHandoffDelayMs),
        setServoPosCommand(
            wristServo, Goal.HANDOFF.wristPos, slowHandoffCommand_wristHandoff2ArmHandoffDelayMs),
        setServoPosCommand(
            slideArmServo,
            Goal.HANDOFF.slideArmPos,
            slowHandoffCommand_ArmHandoff2SlideRetractDelayMs),
        new InstantCommand(() -> slideExtensionVal = Goal.HANDOFF.slideExtension),
        new WaitUntilCommand(this::slideMotorAtHome));
  }

  public Command fastHandoffCommand() {
    return new SequentialCommandGroup(
        setGoalCommand(Goal.HANDOFF),
        setTurnServoPosCommand(TurnServo.DEG_0, handoffCommand_wristTurn2wristHandoffDelayMs),
        new InstantCommand(() -> wristServo.setPosition(Goal.HANDOFF.wristPos)),
        new InstantCommand(() -> slideArmServo.setPosition(Goal.HANDOFF.slideArmPos)),
        new InstantCommand(() -> slideExtensionVal = Goal.HANDOFF.slideExtension),
        new WaitUntilCommand(this::slideMotorAtHome));
  }

  public Command handoffCommand() {
    return new ConditionalCommand(
        slowHandoffCommand(), fastHandoffCommand(), this::slideMotorAtHome);
  }

  public Command swipeCommand() {
    return new SequentialCommandGroup(
        setGoalCommand(Goal.AUTOSWIPE),
        setTurnServoPosCommand(TurnServo.DEG_INVERTED_HORIZ, 0),
        setServoPosCommand(wristServo, Goal.AUTOSWIPE.wristPos, swipeCommand_wrist2ExtendDelayMs),
        new InstantCommand(
            () -> {
              forwardSlideExtension(Goal.AUTOSWIPE.slideExtension);
              slideArmServo.setPosition(Goal.AUTOSWIPE.slideArmPos);
              intakeClawServo.setPosition(Goal.AUTOSWIPE.clawAngle);
            }),
        new WaitUntilCommand(this::slideMotorAtGoal));
  }

  public void openIntakeClaw() {
    intakeClawServo.setPosition(Goal.AIM.clawAngle);
  }

  public void closeIntakeClaw() {
    intakeClawServo.setPosition(Goal.GRAB.clawAngle);
  }

  /** Up to avoid the collision with the clip */
  public void wristUp() {
    wristServo.setPosition(0.65);
  }

  public void wristDown() {
    wristServo.setPosition(0.05);
  }

  public void slideArmDown() {
    // This is down for stowing the liftArm when scoring the speciemen
    slideArmServo.setPosition(Goal.AIM.slideArmPos);
  }

  public void slideArmUp() {
    // This is up for the auto
    slideArmServo.setPosition(Goal.AIM.slideArmPos);
  }

  public void stow() {
    slideArmServo.setPosition(Goal.HANDOFF.slideArmPos);
    wristServo.setPosition(Goal.HANDOFF.wristPos);
  }

  @Config
  public enum Goal {
    STOW(0, 0, 0.2, IntakeClawServo_OPEN),
    AIM(slideExtensionVal, SlideArmServo_AIM_, 0.805, IntakeClawServo_OPEN),
    GRAB(slideExtensionVal, 0.42, 0.805, IntakeClawServo_GRAB),
    HANDOFF(0, 0.75, 0.25, IntakeClawServo_GRAB),
    AUTOSWIPE(SlideMotor_extensionValue, 0.3, 0.45, IntakeClawServo_OPEN);

    public final double slideExtension;
    public final double slideArmPos;
    public final double wristPos;
    public final double clawAngle;

    Goal(double slideExtension, double slideArmPos, double wristPos, double clawAngle) {
      this.slideExtension = slideExtension;
      this.slideArmPos = slideArmPos;
      this.wristPos = wristPos;
      this.clawAngle = clawAngle;
    }
  }

  public void forwardSlideExtension() {
    forwardSlideExtension(SlideMotor_extensionValue);
  }

  public void forwardSlideExtension(double slideExtension) {
    slideExtensionVal = slideExtension;
  }

  public void backwardSlideExtension() {
    slideExtensionVal = 0;
  }

  public void leftTurnServo() {
    switch (turnServo) {
      case DEG_0:
        turnServo = TurnServo.DEG_05;
        break;
      case DEG_05:
        turnServo = TurnServo.DEG_08;
        break;
      case DEG_08:
        turnServo = TurnServo.DEG_08;
        break;
    }
    setServoPos(turnServo);
  }

  public void rightTurnServo() {
    switch (turnServo) {
      case DEG_0:
        turnServo = TurnServo.DEG_0;
        break;
      case DEG_05:
        turnServo = TurnServo.DEG_0;
        break;
      case DEG_08:
        turnServo = TurnServo.DEG_05;
        break;
    }
    setServoPos(turnServo);
  }

  public void setServoPos(TurnServo pos) {
    turnAngleDeg = pos.turnAngleDeg;
    turnServo = pos;
  }

  private Command setTurnServoPosCommand(TurnServo pos, long delay) {
    return new ConditionalCommand(
        new InstantCommand(
                () -> {
                  setServoPos(pos);
                })
            .andThen(new WaitCommand(delay)),
        new InstantCommand(() -> {}),
        () -> getServoPos() != pos);
  }

  public TurnServo getServoPos() {
    return turnAngleDeg == turnServo.turnAngleDeg ? turnServo : TurnServo.UNKNOWN;
  }

  private Command setServoPosCommand(Servo servo, double pos, long delay) {
    return new ConditionalCommand(
        new InstantCommand(
                () -> {
                  servo.setPosition(pos);
                })
            .andThen(new WaitCommand(delay)),
        new InstantCommand(() -> {}),
        () -> servo.getPosition() != pos);
  }

  public enum TurnServo {
    DEG_0(0.25),
    DEG_05(0.4),
    DEG_08(0.8),
    DEG_INVERTED_HORIZ(0.925),
    UNKNOWN(-1);
    public final double turnAngleDeg;

    TurnServo(double setpoint) {
      turnAngleDeg = setpoint;
    }
  }

  public boolean slideMotorAtGoal() {
    return MathUtils.isNear(
        goal.slideExtension, getCurrentPosition(), SlideMotor_atSetPointTolerance);
  }

  public boolean slideMotorAtHome() {
    return MathUtils.isNear(0, getCurrentPosition(), SlideMotor_atSetPointTolerance);
  }

  public long getCurrentPosition() {
    return slideMotor.getCurrentPosition();
  }

  public double getResetPower() {
    return resetPower;
  }

  @Override
  public void runOpenLoop(double percent) {
    double output = Range.clip(percent, -1, 1);
    slideMotor.setPower(output);
  }

  public void resetEncoder() {
    runOpenLoop(0);
    pidController.reset();
    pidController.calculate(0);
    // TODO: does this work?
    slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  //  public Command resetCommand() {
  //    return new StartEndCommand(
  //            () -> {
  //              runOpenLoop();
  //              isResetting = true;
  //            },
  //            () -> {
  //
  //              isResetting = false;
  //            },
  //            this);
  //  }

  //  public boolean atHome() {
  //    return MathUtils.isNear(getCurrentPosition(), 0, 5);
  //  }

  @Override
  public void periodic() {
    wristTurnServo.setPosition(Range.clip(turnAngleDeg, 0, 1));

    slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    double setpointTicks = slideExtensionVal;

    telemetry.addData("Current Goal", goal);
    telemetry.addData("Goal Extension", setpointTicks);
    telemetry.addData("Slide.CurrentPosition", getCurrentPosition());
    telemetry.update();

    double pidPower = pidController.calculate(getCurrentPosition(), setpointTicks);
    pidPower *= 12 / batteryVoltageSensor.getVoltage();
    if (!isResetting) slideMotor.setPower(Range.clip(pidPower, -1, 1));
  }

  public void setServoController(boolean enable) {
    if (enable) {
      intakeClawServo.getController().pwmEnable();
      wristTurnServo.getController().pwmEnable();
      wristTurnServo.getController().pwmEnable();
      slideArmServo.getController().pwmEnable();
    } else {
      intakeClawServo.getController().pwmDisable();
      wristTurnServo.getController().pwmDisable();
      wristTurnServo.getController().pwmDisable();
      slideArmServo.getController().pwmDisable();
    }
  }
}
