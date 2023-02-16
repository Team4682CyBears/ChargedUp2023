package frc.robot.commands;


import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;


public class ArmToLocation extends ArmToPointCommand {

    
   

    public ArmToLocation(ArmSubsystem arm, ArmLocation location) {
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

    enum ArmLocation{
        ARM_STOW,
        ARM_GRAB,
        ARM_LOW_SCORE,
        ARM_MED_SCORE,
        ARM_HIGH_SCORE,
    }
    
}
