package org.firstinspires.ftc.teamcode.robot.stateMachines;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.robot.subsystems.slide;

public class SlideState implements StateMachine {

    slide slides;
    STATE currentState = STATE.MANUAL_CONTROL;

    public enum STATE {
        MANUAL_CONTROL,
        RUN_WITH_PID,
        RUN_TO_POSITION
    }

    public enum EVENT {
        ENABLE_MANUAL,
        ENABLE_PID,
        ENABLE_RTP
    }

    public SlideState(slide slides) {
        this.slides = slides;
    }

    public SlideState.STATE getState() {
        return currentState;
    }

    public void transition(SlideState.EVENT event) {
        switch (event) {
            case ENABLE_MANUAL:
                slides.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slides.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                currentState = STATE.MANUAL_CONTROL;
            case ENABLE_PID:
                slides.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slides.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //currentState = SLIDESTATE.RUN_WITH_PID;
                break;
            case ENABLE_RTP:
                slides.leftMotor.setTargetPosition(slides.getTargetPosition());
                slides.rightMotor.setTargetPosition(slides.getTargetPosition());
                slides.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slides.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                currentState = STATE.RUN_TO_POSITION;
        }
    }

    public void update() {
        switch (currentState) {
            case RUN_WITH_PID:
                //slides.setPIDPower();
                break;
            case RUN_TO_POSITION:
                slides.rightMotor.setTargetPosition(slides.getTargetPosition());
                slides.leftMotor.setTargetPosition(slides.getTargetPosition());
                break;
            default:
                break;
        }
    }

}