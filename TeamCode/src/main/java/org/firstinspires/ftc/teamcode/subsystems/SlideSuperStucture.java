package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import lombok.Getter;
import lombok.Setter;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

public class SlideSuperStucture extends SubsystemBase {
  private final Servo intakeClawServo, wristServo, wristTurnServo;
  private final Servo slideArmServo;
  private final DcMotorEx slideMotor;

  private final PIDController pidController;
  private final double kP = 0.04, kI = 0.0, kD = 0.0;

  private boolean hasGamepiece = false;
  private static double slideExtensionVal = 0;

  private static double turnAngleDeg = 0.8;
  private TurnServo turnServo = TurnServo.DEG_0;

  @Setter @Getter private Goal goal = Goal.STOW;

  private final Telemetry telemetry; // 0 0.5 0.8

  @Getter @Setter private boolean normalHandoff = false;

  public SlideSuperStucture(final HardwareMap hardwareMap, final Telemetry telemetry) {
    slideArmServo = hardwareMap.get(Servo.class, "slideArmServo"); // 0.5 up 0.9 half 1 down

    intakeClawServo = hardwareMap.get(Servo.class, "intakeClawServo"); // 0.3 close 0.7 open

    wristServo = hardwareMap.get(Servo.class, "wristServo"); // 0.05 up 0.75 down

    wristTurnServo = hardwareMap.get(Servo.class, "wristTurnServo");
    wristTurnServo.setDirection(Servo.Direction.REVERSE);

    slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
    slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    pidController = new PIDController(kP, kI, kD);

    this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    goal = Goal.STOW;
  }

  public Command aimCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> goal = Goal.AIM),
        new InstantCommand(
            () -> {
              turnAngleDeg = 0.8;
              turnServo = TurnServo.DEG_0;
            }),
        new WaitCommand(100),
        new InstantCommand(() -> slideArmServo.setPosition(Goal.AIM.slideArmPos)),
        new WaitCommand(100),
        new InstantCommand(() -> wristServo.setPosition(Goal.AIM.wristPos)),
        new InstantCommand(() -> intakeClawServo.setPosition(Goal.AIM.clawAngle)));
  }

  public Command grabCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> goal = Goal.GRAB),
        new InstantCommand(() -> slideArmServo.setPosition(Goal.GRAB.slideArmPos)),
        new WaitCommand(100),
        new InstantCommand(() -> intakeClawServo.setPosition(Goal.GRAB.clawAngle)),
        new WaitCommand(100),
        new InstantCommand(() -> slideArmServo.setPosition(0.3)),
        new InstantCommand(() -> goal = Goal.AIM));
  }

  public Command handoffCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> goal = Goal.HANDOFF),
        new InstantCommand(
            () -> {
              turnAngleDeg = 0.8;
              turnServo = TurnServo.DEG_0;
            }),
        new WaitCommand(100),
        new InstantCommand(() -> wristServo.setPosition(Goal.HANDOFF.wristPos)),
        new WaitCommand(200),
        new InstantCommand(() -> slideArmServo.setPosition(Goal.HANDOFF.slideArmPos)),
        new WaitCommand(300),
        new InstantCommand(() -> slideExtensionVal = Goal.HANDOFF.slideExtension),
        new WaitUntilCommand(this::slideMotorAtGoal));
  }

  public void openIntakeClaw() {
    intakeClawServo.setPosition(0.6);
  }

  public void closeIntakeClaw() {
    intakeClawServo.setPosition(0.36);
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
    slideArmServo.setPosition(0.45);
  }

  public void slideArmUp() {
    // This is up for the auto
    slideArmServo.setPosition(0.35);
  }

  public enum Goal {
    STOW(0, 0, 0, 0, 0.6),
    AIM(slideExtensionVal, 0.45, 0.65, turnAngleDeg, 0.6),
    GRAB(slideExtensionVal, 0.53, 0.65, turnAngleDeg, 0.36),
    HANDOFF(0, 0.25, 0.175, 0, 0.36);

    private final double slideExtension;
    private final double slideArmPos;
    private final double wristPos;
    private final double turnAngle;
    private final double clawAngle;

    Goal(
        double slideExtension,
        double slideArmPos,
        double wristPos,
        double turnAngle,
        double clawAngle) {
      this.slideExtension = slideExtension;
      this.slideArmPos = slideArmPos;
      this.wristPos = wristPos;
      this.turnAngle = turnAngle;
      this.clawAngle = clawAngle;
    }
  }

  public void forwardSlideExtension() {
    slideExtensionVal = 460;
  }

  public void backwardSlideExtension() {
    slideExtensionVal = 0;
  }

  public void leftTurnServo() {
    switch (turnServo) {
      case DEG_0:
        turnAngleDeg = 0.6;
        turnServo = TurnServo.DEG_05;
        break;
      case DEG_05:
        turnAngleDeg = 0.2;
        turnServo = TurnServo.DEG_08;
        break;
      case DEG_08:
        turnAngleDeg = 0.2;
        turnServo = TurnServo.DEG_08;
        break;
    }
  }

  public void rightTurnServo() {
    switch (turnServo) {
      case DEG_0:
        turnAngleDeg = 0.8;
        turnServo = TurnServo.DEG_0;
        break;
      case DEG_05:
        turnAngleDeg = 0.8;
        turnServo = TurnServo.DEG_0;
        break;
      case DEG_08:
        turnAngleDeg = 0.6;
        turnServo = TurnServo.DEG_05;
        break;
    }
  }

  enum TurnServo {
    DEG_0,
    DEG_05,
    DEG_08
  }

  private boolean slideMotorAtGoal() {
    return MathUtils.isNear(goal.slideExtension, slideMotor.getCurrentPosition(), 10);
  }

  @Override
  public void periodic() {
    wristTurnServo.setPosition(Range.clip(turnAngleDeg, 0, 1));

    slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    double setpointTicks = slideExtensionVal;

    telemetry.addData("Current Goal", goal);
    telemetry.addData("Goal Extension", setpointTicks);
    telemetry.update();

    double pidPower = pidController.calculate(slideMotor.getCurrentPosition(), setpointTicks);
    slideMotor.setPower(Range.clip(pidPower, -1, 1));
  }
}
