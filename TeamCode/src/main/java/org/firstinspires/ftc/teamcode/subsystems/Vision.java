package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Vision extends SubsystemBase {
  private final Limelight3A camera;

  private final Servo led;

  public static double ledPWM = 0;

  @Getter private boolean isDataOld = false;
  @Getter @Setter private SampleColor detectionColor = SampleColor.BLUE;
  @Getter private LLResult result;

  private static final double CAMERA_HEIGHT = 307.0;
  private static final double CAMERA_ANGLE = -45.0;
  private static final double TARGET_HEIGHT = 19.05;

  private static final double strafeAlignment = 6.6667;


  private  static final double strafeConversion = 0.4;

  Telemetry telemetry;

  public Vision(final HardwareMap hardwareMap, Telemetry telemetry) {
    camera = hardwareMap.get(Limelight3A.class, "limelight");
    led = hardwareMap.get(Servo.class, "LED");
    this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    for(double i = 0; i <= 1.0; i += 0.1) {
      setLEDPWM(i);
    }

    setLEDPWM(ledPWM);
  }

  public void setLEDPWM(double positionPWM) {
    led.setPosition(positionPWM);
  }


  public void initializeCamera() {
    camera.setPollRateHz(50);
    camera.start();
  }

  @RequiredArgsConstructor
  public enum SampleColor {
    RED(0.0),
    BLUE(1.0),
    YELLOW(2.0);

    private final double colorVal;
  }

  public double getTx(double defaultValue) {
    if (result == null) {
      return defaultValue;
    }
    return result.getTx();
  }

  public double getTy(double defaultValue) {
    if (result == null) {
      return defaultValue;
    }
    return result.getTy();
  }

  public boolean isTargetVisible() {
    if (result == null) {
      return false;
    }
    return result.getTa() > 2;
  }

  public double getDistance() {
    double ty = getTy(0.0);
    double angleToGoalDegrees = CAMERA_ANGLE + ty;
    double angleToGoalRadians = Math.toRadians(angleToGoalDegrees); 
    double distanceMM = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);
    return Math.abs(distanceMM);
  }

  public double getStrafeOffset() {
    double tx = getTx(0);
    if (tx != 0) {
      return tx * strafeAlignment - 127;
    }
    return 0;
  }

  @Override
  public void periodic() {
    camera.updatePythonInputs(
        new double[] {detectionColor.colorVal, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    result = camera.getLatestResult();

    if (result != null) {
      long staleness = result.getStaleness();
      // Less than 100 milliseconds old
      isDataOld = staleness >= 100;
      telemetry.addData("Strafe Offset", getStrafeOffset());
      telemetry.update();

//      telemetry.addData("Tx", result.getTx());
//      telemetry.addData("Ty", result.getTy());
//      telemetry.addData("Ta", result.getTa());
      //telemetry.update();
    }
  }
}
