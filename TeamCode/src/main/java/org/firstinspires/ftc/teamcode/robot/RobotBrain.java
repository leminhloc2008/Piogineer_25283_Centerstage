package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robot.subsystems.Subsystem;

import java.util.Arrays;
import java.util.List;

public class RobotBrain implements Subsystem{

    public List<Subsystem> subsystems;

    @Override
    public void update() {
        for (Subsystem system : subsystems) system.update();
    }
}
