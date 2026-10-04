package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Constants;

@TeleOp(name = "first-teleop")
public class Teleop extends CommandOpMode {
    private Robot bot;

    //the below will run when you hit 'init' on the DS
    //here we will set up telemetry and hardwaremap
    //we will also set up your driver's gamepads here
    @Override
    public void initialize() {
        Constants.telemetry = telemetry;
        Constants.hardwareMap = hardwareMap;
        Constants.gamepad1 = new GamepadEx(gamepad1);
        Constants.gamepad2 = new GamepadEx(gamepad2);

        //this line is last, create the robot
        bot = new Bot(Bot.OpMode.TELEOP);
    }
}
