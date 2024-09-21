package org.firstinspires.ftc.teamcode.OpModes.Test;

import static org.firstinspires.ftc.teamcode.Utilities.Utilities.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Todo: To make the OpMode playable please remove the @Disabled annotation.
@Disabled
@TeleOp()
public class SampleIntakeOpMode extends OpMode {

    private ElapsedTime elapsedTime;

    private final double MAX_INTAKE_RUNNING_TIME_MS = 1500;
    private double previousIntakeStartMS = 0;

    Servo intakePullServo;
    CRServo intakeServo;
    DistanceSensor distanceSensor;

    @Override
    public void init() {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        intakePullServo = hardwareMap.get(Servo.class, "intake_pull_servo");
        intakeServo = hardwareMap.get(CRServo.class, "intake_servo");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            releaseIntake();
            intakeCollect();
        }
        else if (gamepad1.dpad_down) {
            intakeDrop();
        }

        double intakeRunningTimeMS = elapsedTime.milliseconds() - previousIntakeStartMS;

        if (MAX_INTAKE_RUNNING_TIME_MS > intakeRunningTimeMS) return;

        if (distanceSensor.getDistance(DistanceUnit.CM) < 15) {
            stopIntake();
            return;
        }

        stopIntake();
        pullIntake();
    }

    public void releaseIntake() {
        intakePullServo.setPosition(0.2);
    }

    public void pullIntake() {
        intakePullServo.setPosition(1);
    }



    public void intakeCollect() {
        intakeServo.setPower(-0.8);
        previousIntakeStartMS = elapsedTime.milliseconds();
    }

    public void intakeDrop() {
        intakeServo.setPower(0.8);
        previousIntakeStartMS = elapsedTime.milliseconds();
    }

    public void stopIntake() {
        intakeServo.setPower(0.0);
    }
}
