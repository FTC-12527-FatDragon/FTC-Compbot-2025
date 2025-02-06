package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import lombok.RequiredArgsConstructor;

@Config
public class LiftClaw extends SubsystemBase {
  public static double LiftArmServo_SCORE_BASKET = 0.1;
  public static double LiftArmServo_STOW = 0.7;
  public static double LiftArmServo_GRAB_WALL = 0.0;
  public static double LiftArmServo_SCORE_CHAMBER = 0.0;
  public static double LiftArmServo_AVOID_COLLISION = 0.0;

  public static double ClawServo_CLOSE = 0.5425;
  public static double ClawServo_OPEN = 0.8;
  private final Servo liftArmServo;
  private final Servo liftClawServo;
  private boolean isClawOpen;

  public LiftClaw(final HardwareMap hardwareMap) {
    liftArmServo = hardwareMap.get(Servo.class, "liftArmServo"); // 0.3 Up 0.7 Down
    liftClawServo = hardwareMap.get(Servo.class, "clawServo"); // 0 Close 0.5 Open
    setServoController(true);
  }

  public void switchLiftClaw() {
    if (isClawOpen) {
      liftClawServo.setPosition(ClawServo_CLOSE);
    } else {
      liftClawServo.setPosition(ClawServo_OPEN);
    }
    isClawOpen = !isClawOpen;
  }

  public void openClaw() {
    liftClawServo.setPosition(ClawServo_OPEN);
  }

  public void closeClaw() {
    liftClawServo.setPosition(ClawServo_CLOSE);
  }

  public Command openClawCommand() {
    return new InstantCommand(this::openClaw);
  }

  public Command closeClawCommad() {
    return new InstantCommand(this::closeClaw);
  }

  public Command switchClawCommand() {
    return new InstantCommand(this::switchLiftClaw);
  }

  public void upLiftArm() {
    liftArmServo.setPosition(LiftArmServo_SCORE_BASKET);
  }

  public void foldLiftArm() {
    liftArmServo.setPosition(LiftArmServo_STOW);
  }

  public Command setLiftClawServo(LiftClawState state) {
    return new InstantCommand(() -> liftClawServo.setPosition(state.servoPos));
  }

  @RequiredArgsConstructor
  public enum LiftClawState {
    STOW(LiftArmServo_STOW),
    GRAB_FROM_WALL(LiftArmServo_GRAB_WALL),
    SCORE_BASKET(LiftArmServo_SCORE_BASKET),
    AVOID_COLLISION(LiftArmServo_AVOID_COLLISION),
    SCORE_CHAMBER(LiftArmServo_SCORE_CHAMBER);

    private final double servoPos;
  }

  public void setServoController(boolean enable) {
    if (enable) {
      liftArmServo.getController().pwmEnable();
      liftClawServo.getController().pwmEnable();
    } else {
      liftArmServo.getController().pwmDisable();
      liftClawServo.getController().pwmDisable();
    }
  }
}
