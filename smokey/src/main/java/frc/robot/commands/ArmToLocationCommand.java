// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ArmToLocationCommand.java
// Intent: Forms a command to move the dual part arm to defined 'named' spot in space.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToLocationCommand extends ArmToPointCommand {

    /**
     * A command capable to push the arm to a point in space based on the points 'well known name'
     * @param arm - the arm subsystem that should be in use
     * @param location - the enum 'named' location to drive the arm to in space
     */
    public ArmToLocationCommand(ArmSubsystem arm, ArmLocation location) {
        super(arm, 0.0, 0.0);
        
        switch(location){
            case ARM_STOW:
                super.setYValue(Constants.armPresetPositionStowMetersY);
                super.setZValue(Constants.armPresetPositionStowMetersZ);
                break;

            case ARM_GRAB:
                super.setYValue(Constants.armPresetPositionGrabMetersY);
                super.setZValue(Constants.armPresetPositionGrabMetersZ);
                break;

            case ARM_LOW_SCORE:
                super.setYValue(Constants.armPresetPositionScoreLowMetersY);
                super.setYValue(Constants.armPresetPositionScoreLowMetersZ);
                break;

            case ARM_MED_SCORE:
                super.setYValue(Constants.armPresetPositionScoreMediumMetersY);
                super.setYValue(Constants.armPresetPositionScoreMediumMetersZ);
                break;

            case ARM_HIGH_SCORE: 
                super.setYValue(Constants.armPresetPositionScoreHighMetersY);
                super.setYValue(Constants.armPresetPositionScoreHighMetersZ);
                break;

            default: //use stow position
                super.setYValue(Constants.armPresetPositionStowMetersY);
                super.setZValue(Constants.armPresetPositionStowMetersZ);
                break;
        }        
    }


    /**
     * The preset spots the arm should normally be driven to
     */
    public enum ArmLocation{
        ARM_STOW,
        ARM_GRAB,
        ARM_LOW_SCORE,
        ARM_MED_SCORE,
        ARM_HIGH_SCORE,
    }    
}
