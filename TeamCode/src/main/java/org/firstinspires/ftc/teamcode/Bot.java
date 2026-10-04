package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Bot extends Robot {
    public static String mode;
    private Command autoCommand;
    private BotContainer container;
    private Boolean useContainer = true;

    public enum OpMode{
        AUTO,
        TELEOP
    }

    public Bot(OpMode opMode) {
        container = new BotContainer();
        if (opMode == OpMode.TELEOP) {
            initTele();
        } else {
            initAuto();
        }
    }

    public Bot(OpMode opMode, Command auto) { //configure with diff arguments for calling auto
        useContainer = false;
        autoCommand = auto;
        initAuto();
    }

    public void initTele() { //force auto to stop in tele init
        if (autoCommand != null) {
            autoCommand.cancel();
        }
    }

    public void initAuto() {
        if (useContainer) {
            autoCommand = container.getAutoCommand();
        }

        if (autoCommand != null) {
            autoCommand.schedule();
        }
    }
    public static double getVoltage() {
        return Constants.hardwareMap.voltageSensor.get("Control Hub").getVoltage();
    }
}