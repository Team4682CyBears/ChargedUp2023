// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ArmPlusPickerUptakeCommand.java
// Intent: Forms a command to move the dual part arm with y and z inputs.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.common.ChargedUpGamePiece;
import frc.robot.control.ArmPlusPickerAction;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.EveryBotPickerSubsystem;
import frc.robot.subsystems.PowerDistributionPanelWatcherSubsystem;

/*
 * Forms a command to move the dual part arm and the every bot picker as a unit to help in pickup
 */
public class ArmPlusPickerUptakeCommand extends CommandBase {

    private static final double coneInitialSpeed = -0.75;
    private static final double coneSteadyStateSpeed = -1.0;
    private static final double coneOverCurrentInitialLimitAmps = 60.0;
    private static final double coneOverCurrentSteadyStateLimitAmps = 35.5;

    private static final double cubeInitialSpeed = 0.75;
    private static final double cubeSteadyStateSpeed = 1.0;
    private static final double cubeOverCurrentInitialLimitAmps = 60.0;
    private static final double cubeOverCurrentSteadyStateLimitAmps = 35.5;

    private final ArmSubsystem armSubsystem;
    private final EveryBotPickerSubsystem pickerSubsystem;
    private final PowerDistributionPanelWatcherSubsystem watcherSubsystem;
    private ChargedUpGamePiece targetPiece = ChargedUpGamePiece.Cone; 
    private boolean done = false;
    private int currentActionIndex = 0;
    private boolean firstActionStep = true;
    private ArrayList<ArmPlusPickerAction> actionList = new ArrayList<ArmPlusPickerAction>();

    private PowerDistribution distroPannel = null;
    /**
     * The constructor to create a arm movement uptake command.
     * @param theArmSubsystem - the arm subsystem
     * @param thePickerSubsystem - the picker subsystem
     * @param targetGamePiece - the target game piece
     */
    public ArmPlusPickerUptakeCommand(
        ArmSubsystem theArmSubsystem,
        EveryBotPickerSubsystem thePickerSubsystem,
        PowerDistributionPanelWatcherSubsystem watcher,
        ChargedUpGamePiece targetGamePiece) {
        this.armSubsystem = theArmSubsystem;
        this.pickerSubsystem = thePickerSubsystem;
        this.watcherSubsystem = watcher;
        this.targetPiece = targetGamePiece;
        this.distroPannel = watcher.getPowerDistribution();
        addRequirements(this.armSubsystem);
        addRequirements(this.pickerSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        done = false;
        currentActionIndex = 0;
        firstActionStep = true;
        actionList.clear();
        if(this.targetPiece == ChargedUpGamePiece.Cone) {
            this.fillListForCone();
        }
        else {
            this.fillListForCube();
        }
        this.watcherSubsystem.setEnabledWatchOnPort(Constants.EveryBotMotorPdpPortId, false);
    }

    @Override
    public void execute() {
        if(currentActionIndex > actionList.size()-1) {
            this.armSubsystem.setArmSpeeds(0.0, 0.0);
            this.pickerSubsystem.setPickerRelativeSpeed(0.0);
            done = true;
        }
        else {
            double current = distroPannel.getCurrent(Constants.EveryBotMotorPdpPortId);
            ArmPlusPickerAction presentAction = actionList.get(currentActionIndex);
            if(current >= presentAction.getPickerOverCurrentAmps()) {
                this.armSubsystem.setArmSpeeds(0.0, 0.0);
                this.pickerSubsystem.setPickerRelativeSpeed(0.0);
                done = true;
            }
            else if(this.firstActionStep == true) {
                this.armSubsystem.setArmToPointInSpace(presentAction.getYArmPointMeters(), presentAction.getZArmPointMeters());
                this.pickerSubsystem.setPickerRelativeSpeed(presentAction.getPickerMotorSpeed());
                firstActionStep = false;
            } 
            else {
                if(this.armSubsystem.isRequestedArmMovementComplete()) {
                    this.currentActionIndex++;
                    presentAction = actionList.get(currentActionIndex);
                }
                this.armSubsystem.setArmToPointInSpace(presentAction.getYArmPointMeters(), presentAction.getZArmPointMeters());
                this.pickerSubsystem.setPickerRelativeSpeed(presentAction.getPickerMotorSpeed());
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.armSubsystem.setArmSpeeds(0.0, 0.0);
        this.pickerSubsystem.setPickerRelativeSpeed(0.0);
        if(interrupted)
        {
          done = true;
        }
        this.watcherSubsystem.setEnabledWatchOnPort(Constants.EveryBotMotorPdpPortId, true);
    }
    
    @Override
    public boolean isFinished() {
        if(this.done) {
            this.watcherSubsystem.setEnabledWatchOnPort(Constants.EveryBotMotorPdpPortId, true);
        }
        return this.done;
    }

    /**
     * A helper method to prepare for cone steps
     */
    private void fillListForCone() {
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionConeGrabMetersY,
            Constants.armPresetPositionConeGrabMetersZ + 0.07,
            ArmPlusPickerUptakeCommand.coneInitialSpeed,
            ArmPlusPickerUptakeCommand.coneOverCurrentInitialLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionConeGrabMetersY,
            Constants.armPresetPositionConeGrabMetersZ + 0.06,
            ArmPlusPickerUptakeCommand.coneSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.coneOverCurrentInitialLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionConeGrabMetersY,
            Constants.armPresetPositionConeGrabMetersZ + 0.05,
            ArmPlusPickerUptakeCommand.coneSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.coneOverCurrentSteadyStateLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionConeGrabMetersY,
            Constants.armPresetPositionConeGrabMetersZ + 0.04,
            ArmPlusPickerUptakeCommand.coneSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.coneOverCurrentSteadyStateLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionConeGrabMetersY,
            Constants.armPresetPositionConeGrabMetersZ + 0.03,
            ArmPlusPickerUptakeCommand.coneSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.coneOverCurrentSteadyStateLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionConeGrabMetersY,
            Constants.armPresetPositionConeGrabMetersZ + 0.02,
            ArmPlusPickerUptakeCommand.coneSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.coneOverCurrentSteadyStateLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionConeGrabMetersY,
            Constants.armPresetPositionConeGrabMetersZ + 0.01,
            ArmPlusPickerUptakeCommand.coneSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.coneOverCurrentSteadyStateLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionConeGrabMetersY,
            Constants.armPresetPositionConeGrabMetersZ + 0.00,
            ArmPlusPickerUptakeCommand.coneSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.coneOverCurrentSteadyStateLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionConeGrabMetersY,
            Constants.armPresetPositionConeGrabMetersZ - 0.01,
            ArmPlusPickerUptakeCommand.coneSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.coneOverCurrentSteadyStateLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionConeGrabMetersY,
            Constants.armPresetPositionConeGrabMetersZ - 0.02,
            ArmPlusPickerUptakeCommand.coneSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.coneOverCurrentSteadyStateLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionConeGrabMetersY,
            Constants.armPresetPositionConeGrabMetersZ - 0.03,
            ArmPlusPickerUptakeCommand.coneSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.coneOverCurrentSteadyStateLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionConeGrabMetersY,
            Constants.armPresetPositionConeGrabMetersZ - 0.04,
            ArmPlusPickerUptakeCommand.coneSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.coneOverCurrentSteadyStateLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionConeGrabMetersY,
            Constants.armPresetPositionConeGrabMetersZ - 0.05,
            ArmPlusPickerUptakeCommand.coneSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.coneOverCurrentSteadyStateLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionConeGrabMetersY,
            Constants.armPresetPositionConeGrabMetersZ - 0.06,
            ArmPlusPickerUptakeCommand.coneSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.coneOverCurrentSteadyStateLimitAmps));
    }

    /**
     * A helper method to prepare for cube steps
     */
    private void fillListForCube() {
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionCubeGrabMetersY,
            Constants.armPresetPositionCubeGrabMetersZ + 0.07,
            ArmPlusPickerUptakeCommand.cubeInitialSpeed,
            ArmPlusPickerUptakeCommand.cubeOverCurrentInitialLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionCubeGrabMetersY,
            Constants.armPresetPositionCubeGrabMetersZ + 0.06,
            ArmPlusPickerUptakeCommand.cubeSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.cubeOverCurrentInitialLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionCubeGrabMetersY,
            Constants.armPresetPositionCubeGrabMetersZ + 0.05,
            ArmPlusPickerUptakeCommand.cubeSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.cubeOverCurrentSteadyStateLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionCubeGrabMetersY,
            Constants.armPresetPositionCubeGrabMetersZ + 0.04,
            ArmPlusPickerUptakeCommand.cubeSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.cubeOverCurrentSteadyStateLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionCubeGrabMetersY,
            Constants.armPresetPositionCubeGrabMetersZ + 0.03,
            ArmPlusPickerUptakeCommand.cubeSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.cubeOverCurrentSteadyStateLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionCubeGrabMetersY,
            Constants.armPresetPositionCubeGrabMetersZ + 0.02,
            ArmPlusPickerUptakeCommand.cubeSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.cubeOverCurrentSteadyStateLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionCubeGrabMetersY,
            Constants.armPresetPositionCubeGrabMetersZ + 0.01,
            ArmPlusPickerUptakeCommand.cubeSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.cubeOverCurrentSteadyStateLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionCubeGrabMetersY,
            Constants.armPresetPositionCubeGrabMetersZ + 0.00,
            ArmPlusPickerUptakeCommand.cubeSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.cubeOverCurrentSteadyStateLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionCubeGrabMetersY,
            Constants.armPresetPositionCubeGrabMetersZ - 0.01,
            ArmPlusPickerUptakeCommand.cubeSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.cubeOverCurrentSteadyStateLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionCubeGrabMetersY,
            Constants.armPresetPositionCubeGrabMetersZ - 0.02,
            ArmPlusPickerUptakeCommand.cubeSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.cubeOverCurrentSteadyStateLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionCubeGrabMetersY,
            Constants.armPresetPositionCubeGrabMetersZ - 0.03,
            ArmPlusPickerUptakeCommand.cubeSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.cubeOverCurrentSteadyStateLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionCubeGrabMetersY,
            Constants.armPresetPositionCubeGrabMetersZ - 0.04,
            ArmPlusPickerUptakeCommand.cubeSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.cubeOverCurrentSteadyStateLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionCubeGrabMetersY,
            Constants.armPresetPositionCubeGrabMetersZ - 0.05,
            ArmPlusPickerUptakeCommand.cubeSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.cubeOverCurrentSteadyStateLimitAmps));
        actionList.add(new ArmPlusPickerAction(
            Constants.armPresetPositionCubeGrabMetersY,
            Constants.armPresetPositionCubeGrabMetersZ - 0.06,
            ArmPlusPickerUptakeCommand.cubeSteadyStateSpeed,
            ArmPlusPickerUptakeCommand.cubeOverCurrentSteadyStateLimitAmps));
    }

}