package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "teleop")
public class Teleop extends OpMode {
    static Hardware hardware;

    enum States {
        FORWARD,
        BACKWARD,
        STOPPED;


        @NonNull
        @Override
        public String toString() {
            switch (this) {
                case FORWARD:
                    return "FORWARD";
                case BACKWARD:
                    return "BACKWARD";
                case STOPPED:
                    return "STOPPED";
            }
            return "STOPPED";
        }
    }
    public final int reference = 800;
    States state = States.STOPPED;
    Telemetry DashboardTelemetry;
    ElapsedTime time;
    @Override
    public void init() {
        DashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        hardware = new Hardware(hardwareMap);
        time = new ElapsedTime();
    }

    @Override
    public void loop() {
        DashboardTelemetry.addData("state", state);
        DashboardTelemetry.update();
        switch (state) {
            case STOPPED:
                if(gamepad1.y) {
                    state = States.FORWARD;
                    time.reset();
                }
                if(gamepad1.a) {
                    state = States.BACKWARD;
                    time.reset();
                }
                break;
            case FORWARD:
                if(!gamepad1.a) {
                    hardware.liftMotor.setPower(hardware.PIDF(DashboardTelemetry, reference, time));
//                    hardware.move_set_pos(DashboardTelemetry, reference);
                } else {
                    hardware.liftMotor.setPower(0);
                    state = States.STOPPED;
                }
                break;
            case BACKWARD:
                if(!gamepad1.y) {
                    hardware.liftMotor.setPower(hardware.PIDF(DashboardTelemetry, 0, time));
//                    hardware.move_set_pos(DashboardTelemetry, 0);
                } else {
                    hardware.liftMotor.setPower(0);
                    state = States.STOPPED;
                }
                break;
        }
    }

}
