package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class EpicTalonMotor extends SubsystemBase{
    private final WPI_TalonFX mainMotor = new WPI_TalonFX(Constants.JoystickMotorCanId);


public EpicTalonMotor() {
    mainMotor.configFactoryDefault();
}

public void SetSpeed (double incommingSpeed) {
    
    mainMotor.set(ControlMode.PercentOutput, incommingSpeed);
}

public void StopMotor () {
    mainMotor.set(0.0);
}
}