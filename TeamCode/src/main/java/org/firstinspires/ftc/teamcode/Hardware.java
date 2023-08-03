package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Hardware {
    public DcMotorEx liftMotor;

    public static double Kp = 0.002;
    public static double Ki = 0.00002;
    public static double Kd = 0.002;
    public static double Kg = 0.064;
    public static double lastError = 0;

    public static double integralSum = 0;
    public static double a = 1; // a can be anything from 0 < a < 1
    public static double previousEstimate = 0;
    public static double currentEstimate = 0;
    double lastReference;

    public Hardware(HardwareMap map) {
        liftMotor = map.get(DcMotorEx.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public double P(Telemetry telemetry, double reference){
        double position = liftMotor.getCurrentPosition();

        double error = reference - position;

        double out = Kp * error;

        telemetry.addData("reference", reference);
        telemetry.addData("position", position);
        telemetry.addData("error", error);
        telemetry.update();

        return out;
    }

    public double PI(Telemetry telemetry, double reference, ElapsedTime time){
        double position = liftMotor.getCurrentPosition();

        double error = reference - position;
        integralSum = integralSum + (error * time.seconds());

        if(reference != lastReference) integralSum = 0;

        lastReference = reference;
        double out = Kp * error + Ki * integralSum;

        telemetry.addData("reference", reference);
        telemetry.addData("position", position);
        telemetry.addData("error", error);
        telemetry.addData("integral sum", integralSum);
        telemetry.addData("out", out);
        telemetry.update();

        return out;
    }

    public double PID(Telemetry telemetry, double reference, ElapsedTime time){
        double position = liftMotor.getCurrentPosition();

        double error = reference - position;

        double errorChange = (error - lastError);

        currentEstimate = (a * previousEstimate) + (1-a) * errorChange;
        previousEstimate = currentEstimate;

        double derivative = currentEstimate / time.seconds();

        integralSum = integralSum + (error * time.seconds());

        if(reference != lastReference) integralSum = 0;

        lastError = error;

        lastReference = reference;

        time.reset();

        double out = Kp * error + Ki * integralSum + Kd * derivative;

        telemetry.addData("reference", reference);
        telemetry.addData("position", position);
        telemetry.addData("error", error);
        telemetry.addData("error change", errorChange);
        telemetry.addData("derivative", derivative);
        telemetry.addData("integral sum", integralSum);
        telemetry.addData("out", out);
        telemetry.update();

        return out;
    }

    public double PIDF(Telemetry telemetry, double reference, ElapsedTime time){
        double position = liftMotor.getCurrentPosition();

        double error = reference - position;

        double errorChange = (error - lastError);

        currentEstimate = (a * previousEstimate) + (1-a) * errorChange;
        previousEstimate = currentEstimate;

        double derivative = currentEstimate / time.seconds();

        integralSum = integralSum + (error * time.seconds());

        if(reference != lastReference) integralSum = 0;

        lastError = error;

        lastReference = reference;

        time.reset();

        double out = Kp * error + Ki * integralSum + Kd * derivative + Kg;

        telemetry.addData("reference", reference);
        telemetry.addData("position", position);
        telemetry.addData("error", error);
        telemetry.addData("error change", errorChange);
        telemetry.addData("derivative", derivative);
        telemetry.addData("integral sum", integralSum);
        telemetry.addData("out", out);
        telemetry.update();

        return out;
    }

    public void move_set_pos(Telemetry telemetry, int reference){
        double position = liftMotor.getCurrentPosition();

        liftMotor.setTargetPosition(reference);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1);

        telemetry.addData("reference", reference);
        telemetry.addData("position", position);
        telemetry.addData("error", reference - position);
    }
}
