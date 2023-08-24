package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import java.util.ArrayList;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class left extends LinearOpMode {

  private DcMotor E1;
  private DcMotor D1;
  private DcMotor E2;
  private DcMotor D2;
  private DcMotor Linear1;
  private Servo C2_garra;
  private Servo Ex0_dispenser;
  private DcMotor Ex0_E1;
  private DcMotor C1_base;
  private DcMotor C2_intake;
  private Servo Ex1_trava;
  private Servo C0_garra;
  private TouchSensor Sensor_toque;
  private TouchSensor Sensor_toque_INTAKE;
  private Servo C1_pinca;
  private ColorSensor sensor_cor;
  private DistanceSensor distanceSensor;
  private Servo Ex2_guia;
  private DcMotor leds;

  double distanceInCentimeters;
  double ticks = 560;
  double NewTarget;
  int stage = 1;
  int stagePrompt = 1;
  int stagePark = 1;
  int aux = 0;
  double servo_valor;

  OpenCvCamera camera;
  AprilTagDetectionPipeline aprilTagDetectionPipeline;

  static final double FEET_PER_METER = 3.28084;

  double fx = 578.272;
  double fy = 578.272;
  double cx = 402.145;
  double cy = 221.506;

  double tagsize = 0.166;

  int LEFT = 7;
  int MIDDLE = 8;
  int RIGHT = 9;

  AprilTagDetection tagOfInterest = null;

  @Override
  public void runOpMode() {
    C0_garra = hardwareMap.get(Servo.class, "C0_garra");
    C0_garra.setDirection(Servo.Direction.REVERSE);

    Ex1_trava = hardwareMap.get(Servo.class, "Ex1_trava");

    Ex2_guia = hardwareMap.get(Servo.class, "Ex2_guia");

    C2_garra = hardwareMap.get(Servo.class, "C2_garra");
    C2_garra.setDirection(Servo.Direction.REVERSE);

    C1_pinca = hardwareMap.get(Servo.class, "C1_pinca");

    C2_intake = hardwareMap.get(DcMotor.class, "C2_intake");
    C2_intake.setDirection(DcMotor.Direction.REVERSE);
    C2_intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    Ex0_dispenser = hardwareMap.get(Servo.class, "Ex0_dispenser");
    Ex0_dispenser.setDirection(Servo.Direction.REVERSE);

    C1_base = hardwareMap.get(DcMotor.class, "C1_base");
    C1_base.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    C1_base.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    C1_base.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    leds = hardwareMap.get(DcMotor.class, "leds");

    Linear1 = hardwareMap.get(DcMotor.class, "C0_linear");
    Linear1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Linear1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Linear1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Linear1.setDirection(DcMotor.Direction.REVERSE);

    E1 = hardwareMap.get(DcMotor.class, "Ex0_E1");
    E1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    E1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    E2 = hardwareMap.get(DcMotor.class, "Ex1_E2");
    E2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    E2.setDirection(DcMotor.Direction.REVERSE);
    E2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    D1 = hardwareMap.get(DcMotor.class, "Ex2_D1");
    D1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    D1.setDirection(DcMotor.Direction.REVERSE);
    D1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    D2 = hardwareMap.get(DcMotor.class, "Ex3_D2");
    D2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    D2.setDirection(DcMotor.Direction.REVERSE);
    D2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    Sensor_toque = hardwareMap.touchSensor.get("C1_sensor_toque");
    Sensor_toque_INTAKE = hardwareMap.touchSensor.get("C3_sensor_toque");

    sensor_cor = hardwareMap.get(ColorSensor.class, "cor1");

    D1.setDirection(DcMotor.Direction.REVERSE);
    E1.setDirection(DcMotor.Direction.FORWARD);

    C2_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    Ex0_dispenser.setPosition(0.6);
    C1_pinca.setPosition(0);

    int cameraMonitorViewId = hardwareMap.appContext
      .getResources()
      .getIdentifier(
        "cameraMonitorViewId",
        "id",
        hardwareMap.appContext.getPackageName()
      );
    camera =
      OpenCvCameraFactory
        .getInstance()
        .createWebcam(
          hardwareMap.get(WebcamName.class, "Webcam 1"),
          cameraMonitorViewId
        );
    aprilTagDetectionPipeline =
      new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

    camera.setPipeline(aprilTagDetectionPipeline);
    camera.openCameraDeviceAsync(
      new OpenCvCamera.AsyncCameraOpenListener() {
        @Override
        public void onOpened() {
          camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
        }

        @Override
        public void onError(int errorCode) {}
      }
    );

    telemetry.setMsTransmissionInterval(50);

    /*
     * The INIT-loop:
     * This REPLACES waitForStart!
     */
    while (!isStarted() && !isStopRequested()) {
      ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

      if (currentDetections.size() != 0) {
        boolean tagFound = false;

        for (AprilTagDetection tag : currentDetections) {
          if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
            tagOfInterest = tag;
            tagFound = true;
            break;
          }
        }

        if (tagFound) {
          telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
          tagToTelemetry(tagOfInterest);
        } else {
          telemetry.addLine("Don't see tag of interest :(");

          if (tagOfInterest == null) {
            telemetry.addLine("(The tag has never been seen)");
          } else {
            telemetry.addLine(
              "\nBut we HAVE seen the tag before; last seen at:"
            );
            tagToTelemetry(tagOfInterest);
          }
        }
      } else {
        telemetry.addLine("Don't see tag of interest :(");

        if (tagOfInterest == null) {
          telemetry.addLine("(The tag has never been seen)");
        } else {
          telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
          tagToTelemetry(tagOfInterest);
        }
      }

      telemetry.update();
      sleep(20);
    }

    if (tagOfInterest != null) {
      telemetry.addLine("Tag snapshot:\n");
      tagToTelemetry(tagOfInterest);
      telemetry.update();
    } else {
      telemetry.addLine(
        "No tag snapshot available, it was never sighted during the init loop :("
      );
      telemetry.update();
    }

    // Loop de inicalização
    while (opModeIsActive()) {
      leds.setPower(0.5);
      /* Actually do something useful */
      distanceSensor = (DistanceSensor) sensor_cor;
      distanceInCentimeters = distanceSensor.getDistance(DistanceUnit.CM);
      if (tagOfInterest == null) {
        //default trajectory here if preferred
      } else if (tagOfInterest.id == LEFT) {
        run(2000, 800);
      } else if (tagOfInterest.id == MIDDLE) {
        run(2000, -450);
      } else {
        run(2000, -1700);
      }
    }
  }

  void run(int v1, int v2) {
    if (stage == 1) {
      walk(1000, -2977);
      while (D1.isBusy()) {}
      sleep(200);
      walk(1000, 140);
      stage = 2;
    } else if (stage == 2) {
      sleep(300);
      curve(1000, 1020);
      while (D1.isBusy()) {}
      stage = 3;
    } else if (stage == 3) {
      sleep(300);
      walk(900, 450);
      while (D1.isBusy()) {}
      stage = 4;
    } else if (stage == 4) {
      prompt();
      while (Linear1.isBusy()) {}
      stage = 5;
    } else if (stage == 5) {
      walk(v1, v2);
      while (D1.isBusy()) {}
      stage = 6;
    }
  }

  private void walk(double velocity, int rotations) {
    E1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    E2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    D1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    D2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    E1.setTargetPosition(rotations);
    E2.setTargetPosition(rotations);
    D1.setTargetPosition(rotations);
    D2.setTargetPosition(rotations);
    E1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    E2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    D1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    D2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    ((DcMotorEx) E1).setVelocity(velocity);
    ((DcMotorEx) E2).setVelocity(velocity);
    ((DcMotorEx) D1).setVelocity(velocity);
    ((DcMotorEx) D2).setVelocity(velocity);
    while (D1.isBusy()) {}
  }

  private void Lateral(int lateral_velocity, int lateral_rotation) {
    E1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    E2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    D1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    D2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    E1.setTargetPosition(-lateral_rotation);
    E2.setTargetPosition(lateral_rotation);
    D1.setTargetPosition(lateral_rotation);
    D2.setTargetPosition(-lateral_rotation);
    E1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    E2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    D1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    D2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    ((DcMotorEx) E1).setVelocity(lateral_velocity);
    ((DcMotorEx) E2).setVelocity(lateral_velocity);
    ((DcMotorEx) D1).setVelocity(lateral_velocity);
    ((DcMotorEx) D2).setVelocity(lateral_velocity);
    while (D1.isBusy()) {}
  }

  private void curve(double curve_velocity, int curve_rotations) {
    E1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    E2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    D1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    D2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    E1.setTargetPosition(curve_rotations);
    E2.setTargetPosition(curve_rotations);
    D1.setTargetPosition(-curve_rotations);
    D2.setTargetPosition(-curve_rotations);
    E1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    E2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    D1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    D2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    ((DcMotorEx) E1).setVelocity(curve_velocity);
    ((DcMotorEx) E2).setVelocity(curve_velocity);
    ((DcMotorEx) D1).setVelocity(curve_velocity);
    ((DcMotorEx) D2).setVelocity(curve_velocity);
    while (D1.isBusy()) {}
  }

  private void prompt() {
    if (stagePrompt == 1) {
      linearSubir(0.75, 450);
      while (Linear1.isBusy()) {}
      linearSubir(0.79, 650);
      while (Linear1.isBusy()) {}
      ultimoCone();
      stagePrompt = 2;
    }
  }

  private void ultimoCone() {
    sleep(100);
    if (distanceInCentimeters < 6) {
      Ex0_dispenser.setPosition(0.6);
      linearAlinhar(297);
      while (C1_base.isBusy()) {}
      Ex1_trava.setPosition(0.4);
      Ex2_guia.setPosition(1);
      enconderSubir(3.5);
      while (Linear1.isBusy()) {}
      Ex0_dispenser.setPosition(0);
      sleep(200);
      Ex1_trava.setPosition(0.7);
    } else {
      sleep(200);
      if (distanceInCentimeters < 6) {
        Ex0_dispenser.setPosition(0.6);
        linearAlinhar(297);
        while (C1_base.isBusy()) {}
        Ex1_trava.setPosition(0.4);
        enconderSubir(3.5);
        while (Linear1.isBusy()) {}
        Ex0_dispenser.setPosition(0);
        sleep(200);
        Ex1_trava.setPosition(0.7);
        Ex2_guia.setPosition(1);
      }
    }
    sleep(250);
    Ex2_guia.setPosition(0);
    Ex0_dispenser.setPosition(0.77);
    while (Linear1.isBusy()) {}
    if (Sensor_toque.isPressed() == false) {
      enconderDescer(0);
      while (Linear1.isBusy()) {}
      linearAlinharVoltar(0);
    }
  }

  private void linearSubir(double valorServo, int sleep) {
    C2_intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    C2_garra.setPosition(valorServo);
    C0_garra.setPosition(valorServo);
    sleep(150);
    if (distanceInCentimeters < 6) {
      Ex0_dispenser.setPosition(0.6);
      linearAlinhar(297);
      while (C1_base.isBusy()) {}
      Ex1_trava.setPosition(0.4);
      enconderSubir(3.4);
      Ex2_guia.setPosition(1);
      while (Linear1.isBusy()) {}
      Ex0_dispenser.setPosition(0);
      sleep(200);
      Ex1_trava.setPosition(0.7);
      Ex2_guia.setPosition(1);
      C2_intake.setPower(0.3);
    } else {
      sleep(200);
      if (distanceInCentimeters < 6) {
        Ex0_dispenser.setPosition(0.6);
        linearAlinhar(297);
        while (C1_base.isBusy()) {}
        Ex1_trava.setPosition(0.4);
        enconderSubir(3.5);
        while (Linear1.isBusy()) {}
        Ex0_dispenser.setPosition(0);
        sleep(200);
        Ex1_trava.setPosition(0.7);
        Ex2_guia.setPosition(1);
        C2_intake.setPower(0.3);
      }
    }
    sleep(300);
    Ex0_dispenser.setPosition(0.77);
    while (Linear1.isBusy()) {}
    Ex2_guia.setPosition(0);
    if (Sensor_toque.isPressed() == false) {
      enconderDescer(0);
      while (Linear1.isBusy()) {}
    }
    sleep(200);
    C2_intake.setPower(-0.2);
    sleep(70);
    C2_garra.setPosition(valorServo);
    C0_garra.setPosition(valorServo);
    C2_intake.setPower(0);
    sleep(sleep);
    C1_pinca.setPosition(1);
    sleep(sleep);
    C2_garra.setPosition(0.50);
    C0_garra.setPosition(0.50);
    C2_intake.setPower(0.25);
    sleep(550);
    C2_intake.setPower(0);
    linearAlinharVoltar(0);
    while (C1_base.isBusy()) {}
    if (Sensor_toque_INTAKE.isPressed() == false) {
      intakeVoltar(0);
      while (C2_intake.isBusy()) {}
    } else if (Sensor_toque_INTAKE.isPressed() == true) {
      C2_intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      C2_intake.setPower(0);
    }
    C2_garra.setPosition(0.20);
    C0_garra.setPosition(0.20);
    sleep(300);
    C1_pinca.setPosition(0);
    sleep(300);
    C2_garra.setPosition(0.50);
    C0_garra.setPosition(0.50);
  }

  private void enconderSubir(double turnageS) {
    NewTarget = ticks * turnageS;
    Linear1.setTargetPosition((int) NewTarget);
    Linear1.setPower(1);
    Linear1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  private void enconderDescer(double turnageD) {
    NewTarget = ticks * turnageD;
    Linear1.setTargetPosition((int) NewTarget);
    Linear1.setPower(-1);
    Linear1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  private void intakeIr(int rot) {
    C2_intake.setTargetPosition(rot);
    C2_intake.setPower(1);
    C2_intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  private void intakeVoltar(int rotV) {
    NewTarget = ticks * rotV;
    C2_intake.setTargetPosition((int) NewTarget);
    C2_intake.setPower(-1);
    C2_intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  private void linearAlinhar(int base) {
    C1_base.setTargetPosition(base);
    C1_base.setPower(1);
    C1_base.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  private void linearAlinharVoltar(int base) {
    C1_base.setTargetPosition(base);
    C1_base.setPower(-1);
    C1_base.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  void tagToTelemetry(AprilTagDetection detection) {
    telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    telemetry.addLine(
      String.format(
        "Translation X: %.2f feet",
        detection.pose.x * FEET_PER_METER
      )
    );
    telemetry.addLine(
      String.format(
        "Translation Y: %.2f feet",
        detection.pose.y * FEET_PER_METER
      )
    );
    telemetry.addLine(
      String.format(
        "Translation Z: %.2f feet",
        detection.pose.z * FEET_PER_METER
      )
    );
    telemetry.addLine(
      String.format(
        "Rotation Yaw: %.2f degrees",
        Math.toDegrees(detection.pose.yaw)
      )
    );
    telemetry.addLine(
      String.format(
        "Rotation Pitch: %.2f degrees",
        Math.toDegrees(detection.pose.pitch)
      )
    );
    telemetry.addLine(
      String.format(
        "Rotation Roll: %.2f degrees",
        Math.toDegrees(detection.pose.roll)
      )
    );
  }
}
