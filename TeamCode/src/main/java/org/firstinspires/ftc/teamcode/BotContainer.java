package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.gamepad1;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;


public class BotContainer {
    //begin with all instance gets (these are our subsystems and controllers)
    private GamepadEx Gamepad1 = gamepad1;
    private GamepadEx Gamepad2 = Constants.gamepad2;
    public Drivetrain drivetrain = Drivetrain.getInstance();


    public BotContainer() {
        setDefaultCommands();
        configBindings();
    }

    private void setDefaultCommands(){
        //instructions for driver: right joystick is only left-right rotation, left joystick is for translational movement, and the right bumper enables slow mode
        drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.driveRC(gamepad1.getLeftX(), gamepad1.getLeftY(), gamepad1.getRightX(), gamepad1.getButton(GamepadKeys.Button.RIGHT_BUMPER)), drivetrain));
    }
    private void configBindings() {

    }

    public Command getAutoCommand(){
        return null;
    }

}