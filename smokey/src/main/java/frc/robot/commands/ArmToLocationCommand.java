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
import frc.robot.common.ChargedUpGamePiece;
import frc.robot.control.ManualInputInterfaces;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToLocationCommand extends ArmToPointCommand {

    /**
     * A command capable to push the arm to a point in space based on the points 'well known name'
     * @param arm - the arm subsystem that should be in use
     * @param location - the enum 'named' location to drive the arm to in space
     */
    public ArmToLocationCommand(ArmSubsystem arm, ArmLocation location, ManualInputInterfaces input) {
        super(arm, 0.0, 0.0);
        
        switch(location){
            case ARM_STOW:
                super.setYValue(Constants.armPresetPositionStowMetersY);
                super.setZValue(Constants.armPresetPositionStowMetersZ);
                break;

            case ARM_GRAB:
                if(input.getTargetGamePiece() == ChargedUpGamePiece.Cone) {
                    System.out.println("Setting CONE Grab");
                    super.setYValue(Constants.armPresetPositionConeGrabMetersY);
                    super.setZValue(Constants.armPresetPositionConeGrabMetersZ);
                }
                else if(input.getTargetGamePiece() == ChargedUpGamePiece.Cube) {
                    System.out.println("Setting CUBE Grab");
                    super.setYValue(Constants.armPresetPositionCubeGrabMetersY);
                    super.setZValue(Constants.armPresetPositionCubeGrabMetersZ);
                }
                break;

            case ARM_LOW_SCORE:
                if(input.getTargetGamePiece() == ChargedUpGamePiece.Cone) {
                    super.setYValue(Constants.armPresetPositionConeScoreLowMetersY);
                    super.setZValue(Constants.armPresetPositionConeScoreLowMetersZ);
                }
                else if(input.getTargetGamePiece() == ChargedUpGamePiece.Cube) {
                    super.setYValue(Constants.armPresetPositionCubeScoreLowMetersY);
                    super.setZValue(Constants.armPresetPositionCubeScoreLowMetersZ);
                }
                break;

            case ARM_MED_SCORE:
                if(input.getTargetGamePiece() == ChargedUpGamePiece.Cone) {
                    super.setYValue(Constants.armPresetPositionConeScoreMediumMetersY);
                    super.setZValue(Constants.armPresetPositionConeScoreMediumMetersZ);
                }
                else if(input.getTargetGamePiece() == ChargedUpGamePiece.Cube) {
                    super.setYValue(Constants.armPresetPositionCubeScoreMediumMetersY);
                    super.setZValue(Constants.armPresetPositionCubeScoreMediumMetersZ);
                }
                break;

            case ARM_HIGH_SCORE: 
                if(input.getTargetGamePiece() == ChargedUpGamePiece.Cone) {
                    super.setYValue(Constants.armPresetPositionConeScoreHighMetersY);
                    super.setZValue(Constants.armPresetPositionConeScoreHighMetersZ);
                }
                else if(input.getTargetGamePiece() == ChargedUpGamePiece.Cube) {
                    super.setYValue(Constants.armPresetPositionCubeScoreHighMetersY);
                    super.setZValue(Constants.armPresetPositionCubeScoreHighMetersZ);
                }
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
