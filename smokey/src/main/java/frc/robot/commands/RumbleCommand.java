package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;


public class RumbleCommand extends CommandBase{

    private XboxController controller = new XboxController(0);
    private Timer timer;

    public RumbleCommand(XboxController controller) {
        this.timer = new Timer();
        this.controller = controller;
    }


    @Override
    public void execute() {
        if (controller.getBButton()) {
            if(timer.hasElapsed(Constants.rumbleTime)){
                System.out.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                controller.setRumble(RumbleType.kBothRumble, 1.0);
            } else {
                controller.setRumble(RumbleType.kBothRumble, 0.0);
            }   
        }
    }
    @Override
    public void end(boolean interrupted) {
        controller.setRumble(RumbleType.kBothRumble, 0.0);
    }
    
}