// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: ClimbCommandBuilder.java
// Intent: Forms the key command build logic for the robot to climb in the 'hangar zone' at the end of the match.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.builders;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class ClimbCommandBuilder
{
    /**
     * A builder method to assemble a succession of commands/steps that will attain the medium bar climb
     * Notes (Jonathan to fill in his steps from your notes here!!!):
     * 1. move the jaws out of the way to the intake position
     * 2. extend the climber arms
     * 3. get the angle arms to the ready position
     * @param collection - The grouping of subystems and input content necessary to control various operations in the robot
     * @return The command that represents a succession of commands/steps that form the action associated with this method  
     */
    public static Command buildExtensionAndReadyPosition(SubsystemCollection collection)
    {
        JawsIntake jawsOutOfTheWay = new JawsIntake(collection.getJawsSubsystem());
        TelescopingArmExtendMiddle extendArms = new TelescopingArmExtendMiddle(collection.getTelescopingArmsSubsystem());
        AngleArmsAngleVariable angleArmsToReadyPosition = new AngleArmsAngleVariable(collection.getAngleArmsSubsystem(), Constants.angleArmsReadyPositionAngle);
        ParallelCommandGroup jawsAndExtendGroup = new ParallelCommandGroup(jawsOutOfTheWay, extendArms);
        SequentialCommandGroup overallCommandGroup = new SequentialCommandGroup(jawsAndExtendGroup, angleArmsToReadyPosition);
        return overallCommandGroup;
    }

    /**
     * A builder method to assemble a succession of commands/steps that will attain the medium bar climb
     * Notes (Jonathan to fill in his steps from your notes here!!!):
     * 1. action starts with robot at ground level where and extended telescoping arms have been driven manually to touch mid bar
     * 2. retract telescoping arms to lift robot to medium bar
     * 3. angle arms swing forward until hooks make contact with medium bar
     * @param collection - The grouping of subystems and input content necessary to control various operations in the robot
     * @return The command that represents a succession of commands/steps that form the action associated with this method  
     */
    public static Command buildMediumBarClimb(SubsystemCollection collection)
    {
        TelescopingArmRetract liftRobot = new TelescopingArmRetract(collection.getTelescopingArmsSubsystem());
        JawsAngleVariable angleArmForward = new JawsAngleVariable(collection.getJawsSubsystem(), 90);
        TelescopingArmExtendVariable shiftWeightToAngleArms = new TelescopingArmExtendVariable(collection.getTelescopingArmsSubsystem(), 6);
        SequentialCommandGroup overallCommandGroup = new SequentialCommandGroup(liftRobot, angleArmForward, shiftWeightToAngleArms);
        return overallCommandGroup;
    }

    /**
     * A builder method to assemble a succession of commands/steps that will attain the high bar climb
     * Notes (Jonathan to fill in his steps from your notes here!!!):
     * 1. action starts with robot hanging from the angle arms on the medium bar where there is essentially no weight on the angle arm hooks
     * 2. angle arms swing forwards to shift weight onto angle arms, disconnect telescoping arms, and tilt robot
     * 3. extend telescoping arms until it has extended past high bar
     * 4. angle arms swing backwards to tilt robot until telescoping arms make contact with high bar
     * 5. retract telescoping arms until angle arms are disconnected from middle bar
     * 6. angle arms completely swing back in front of high bar
     * 7. retract telescoping arms up to high bar
     * 8. angle arms swing forward to hook onto high bar
     * @param collection - The grouping of subystems and input content necessary to control various operations in the robot
     * @return The command that represents a succession of commands/steps that form the action associated with this method  
     */
    public static Command buildHighBarClimb(SubsystemCollection collection)
    {
        // After the notes above are done,  the idea is to take the actions described there and convert them into a set of steps that use the building block commands.
        JawsAngleVariable angleArmForwardTilt = new JawsAngleVariable(collection.getJawsSubsystem(), 155);
        TelescopingArmExtendHigh extendArms = new TelescopingArmExtendHigh(collection.getTelescopingArmsSubsystem());
        JawsAngleVariable angleArmBackwardTilt = new JawsAngleVariable(collection.getJawsSubsystem(), 135); 
        TelescopingArmRetract liftRobot = new TelescopingArmRetract(collection.getTelescopingArmsSubsystem());
        JawsAngleVariable angleArmBackward = new JawsAngleVariable(collection.getJawsSubsystem(), 45); 
        JawsAngleVariable angleArmForward = new JawsAngleVariable(collection.getJawsSubsystem(), 90);
        TelescopingArmExtendVariable shiftWeightToAngleArms = new TelescopingArmExtendVariable(collection.getTelescopingArmsSubsystem(), 6);
        ParallelCommandGroup tiltAndExtendGroup = new ParallelCommandGroup(extendArms, angleArmForwardTilt); 
        ParallelCommandGroup liftAndAngleArmGroup = new ParallelCommandGroup(liftRobot, angleArmBackward);  
        SequentialCommandGroup overallCommandGroup = new SequentialCommandGroup(tiltAndExtendGroup, angleArmBackwardTilt, liftAndAngleArmGroup, angleArmForward, shiftWeightToAngleArms);        
        return overallCommandGroup;
    }
    
    /**
     * A builder method to assemble a succession of commands/steps that will attain the traversal bar climb
     * Notes (Jonathan to fill in his steps from your notes here!!!):
     * 1. action starts with robot hanging from the angle arms on the high bar where there is essentially no weight on the angle arm hooks
     * 2. angle arms swing forward to shift weight onto angle arms, disconnect telescoping arms, and tilt robot
     * 3. extend telescoping arms until it has extended past high bar
     * 4. angle arms swing back to tilt robot until telescoping arms make contact with high bar
     * 5. retract telescoping arms to lift robot to high bar
     * @param collection - The grouping of subystems and input content necessary to control various operations in the robot
     * @return The command that represents a succession of commands/steps that form the action associated with this method  
     */
    public static Command buildTraversalBarClimb(SubsystemCollection collection)
    {
        // After the notes above are done,  the idea is to take the actions described there and convert them into a set of steps that use the building block commands.
        JawsAngleVariable angleArmForwardTilt = new JawsAngleVariable(collection.getJawsSubsystem(), 155);
        TelescopingArmExtendHigh extendArms = new TelescopingArmExtendHigh(collection.getTelescopingArmsSubsystem());
        JawsAngleVariable angleArmBackwardTilt = new JawsAngleVariable(collection.getJawsSubsystem(), 135); 
        TelescopingArmRetract liftRobot = new TelescopingArmRetract(collection.getTelescopingArmsSubsystem());
        ParallelCommandGroup tiltAndExtendGroup = new ParallelCommandGroup(extendArms, angleArmForwardTilt);   
        SequentialCommandGroup overallCommandGroup = new SequentialCommandGroup(tiltAndExtendGroup, angleArmBackwardTilt, liftRobot);    
        return overallCommandGroup;
    }

}
