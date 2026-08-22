package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp
public class TuffMode extends CommandOpMode{
    protected Robot robot = new Robot();

    @Override
    public void init() {
        super.init();
        robot.initialize(true, hardwareMap);
    }

    @Override
    public void loop(){
        robot.update(0,0,0);
        robot.shooter.setClose(false);
        robot.shooter.update(100000);
        super.loop();
    }
}
