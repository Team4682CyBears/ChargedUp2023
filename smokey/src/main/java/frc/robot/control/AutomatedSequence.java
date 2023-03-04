// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: AutomatedSequence.java
// Intent: Provides static methods to assemble helpful action sequences.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmToPointCommand;
import frc.robot.commands.EveryBotPickerAutoUptakeCommand;

public class AutomatedSequence {

    private static final double PickupSequenceMaximumDurationSeconds = 3.5;

    /**
     * A method that will return a sequence of commands that will move the arm to above the 
     * pickup position and flow the arm downward while attempting to uptake the game piece
     * @param collection - the subsystems
     * @return a grouping of commands run as a block to perform uptake at the pickup station
     */
    public static Command GetPickupSequence(SubsystemCollection collection) {

        SequentialCommandGroup outerSequence = new SequentialCommandGroup();
        outerSequence.addCommands(
            new ArmToPointCommand(
                collection.getArmSubsystem(),
                Constants.armPresetPositionGrabMetersY,
                Constants.armPresetPositionGrabMetersZ + 0.07));

        ParallelCommandGroup loweringWithPicker = new ParallelCommandGroup(
            new EveryBotPickerAutoUptakeCommand(
                collection.getEveryBotPickerSubsystem()),
            new ArmToPointCommand(
                collection.getArmSubsystem(),
                Constants.armPresetPositionGrabMetersY,
                Constants.armPresetPositionGrabMetersZ + 0.06),
            new ArmToPointCommand(
                collection.getArmSubsystem(),
                Constants.armPresetPositionGrabMetersY,
                Constants.armPresetPositionGrabMetersZ + 0.05),
            new ArmToPointCommand(
                collection.getArmSubsystem(),
                Constants.armPresetPositionGrabMetersY,
                Constants.armPresetPositionGrabMetersZ + 0.04),
            new ArmToPointCommand(
                collection.getArmSubsystem(),
                Constants.armPresetPositionGrabMetersY,
                Constants.armPresetPositionGrabMetersZ + 0.03),
            new ArmToPointCommand(
                collection.getArmSubsystem(),
                Constants.armPresetPositionGrabMetersY,
                Constants.armPresetPositionGrabMetersZ + 0.02),
            new ArmToPointCommand(
                collection.getArmSubsystem(),
                Constants.armPresetPositionGrabMetersY,
                Constants.armPresetPositionGrabMetersZ + 0.01),
            new ArmToPointCommand(
                collection.getArmSubsystem(),
                Constants.armPresetPositionGrabMetersY,
                Constants.armPresetPositionGrabMetersZ + 0.00),
            new ArmToPointCommand(
                collection.getArmSubsystem(),
                Constants.armPresetPositionGrabMetersY,
                Constants.armPresetPositionGrabMetersZ - 0.01),
            new ArmToPointCommand(
                collection.getArmSubsystem(),
                Constants.armPresetPositionGrabMetersY,
                Constants.armPresetPositionGrabMetersZ - 0.02),
            new ArmToPointCommand(
                collection.getArmSubsystem(),
                Constants.armPresetPositionGrabMetersY,
                Constants.armPresetPositionGrabMetersZ - 0.03),
            new ArmToPointCommand(
                collection.getArmSubsystem(),
                Constants.armPresetPositionGrabMetersY,
                Constants.armPresetPositionGrabMetersZ - 0.04),
            new ArmToPointCommand(
                collection.getArmSubsystem(),
                Constants.armPresetPositionGrabMetersY,
                Constants.armPresetPositionGrabMetersZ - 0.05),
            new ArmToPointCommand(
                collection.getArmSubsystem(),
                Constants.armPresetPositionGrabMetersY,
                Constants.armPresetPositionGrabMetersZ - 0.06)
            );

        outerSequence.addCommands(loweringWithPicker);

        return outerSequence.withTimeout(PickupSequenceMaximumDurationSeconds);
    }

}
