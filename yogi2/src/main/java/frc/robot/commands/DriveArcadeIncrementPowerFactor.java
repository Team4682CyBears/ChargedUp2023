// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: DriveArcadeIncrementPowerFactor.java
// Intent: Forms a command to drive the wheels and also adjust the power factor.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.common.PowerFactorIncrementDirection;

public class DriveArcadeIncrementPowerFactor extends CommandBase
{

  private static final double powerFactorIncrement = 0.05;
  private DriveTrain driveTrain;
  private double targetPower = 0.0;
  private double targetSpin = 0.0;
  private PowerFactorIncrementDirection targetDirection = PowerFactorIncrementDirection.Down;
  
  /** 
  * Creates a new DriveArcadeIncrementPowerFactor. 
  * 
  * @param driveTrainSubsystem - the drive train subsystem
  * @param power - the the target spin componet -1.0 to 1.0
  * @param spin - the the target spin componet -1.0 to 1.0
  * @param direction - the target increment direction
  */
  public DriveArcadeIncrementPowerFactor(
    DriveTrain driveTrainSubsystem,
    double power,
    double spin,
    PowerFactorIncrementDirection direction)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrainSubsystem;
    addRequirements(driveTrain);
    targetPower = power;
    targetSpin = spin;
    targetDirection = direction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
      // update the power factor
      if(targetDirection == PowerFactorIncrementDirection.Up)
      {
          this.driveTrain.setArcadePowerFactor(
              this.driveTrain.getArcadePowerFactor() + DriveArcadeIncrementPowerFactor.powerFactorIncrement);
      }
      else if(targetDirection == PowerFactorIncrementDirection.Down)
      {
        this.driveTrain.setArcadePowerFactor(
            this.driveTrain.getArcadePowerFactor() - DriveArcadeIncrementPowerFactor.powerFactorIncrement);
      }

      // do the drive command
      this.driveTrain.arcadeDrive(targetPower, targetSpin);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return true;
  }
}
