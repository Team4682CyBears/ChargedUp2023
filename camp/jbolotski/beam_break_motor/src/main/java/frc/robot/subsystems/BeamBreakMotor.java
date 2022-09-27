package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.MotorUtils;

/* Motor runs at a constant speed while beam break is unbroken
 * Supply default motor speed in constructor
 * Motor stops when beam is broken
 */
public class BeamBreakMotor extends SubsystemBase
{
    private WPI_TalonSRX beamBreakMotor = new WPI_TalonSRX(Constants.beamBreakMotorCanId);
    private double motorSpeed = 0.0;

    /**
     * constructor
     * @param speed default speed for motor to run
     */
    public BeamBreakMotor(double speed)
    {
        beamBreakMotor.configFactoryDefault();
        beamBreakMotor.setNeutralMode(NeutralMode.Brake);
        beamBreakMotor.setInverted(false); 
        motorSpeed = speed; 

        //Note! the SubsystemBase class automatically registers the subsystem
    }

    /**
    * A method exposed to callers to set motor speed.  This method does not run the motor. 
    * use runMotor() method to run the motor at the set speed.   
    */
    public void setMotorSpeed(double speed)
    { 
        motorSpeed = MotorUtils.truncateValue(speed, -1.0, 1.0); 
    }

    /**
     * A method exposed to callers to cause the motor to run at a constant speed.  
     */
    public void runMotor(){
        beamBreakMotor.set(ControlMode.PercentOutput, motorSpeed);
    }

    /**
    * A method exposed to callers to cause motors to stop
    */
    public void stopMotor()
    {
        beamBreakMotor.stopMotor();
    }

    /**
     * returns the beamBreakMotor TalonSRX 
     * needed for simulation.
     * @return WPI_TalonSRX 
     */
    public TalonSRX getController() {
        return beamBreakMotor;
    }

} 
