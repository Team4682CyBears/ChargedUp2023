package frc.robot.swerveHelpers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.swervedrivespecialties.swervelib.ctre.CanCoderAbsoluteConfiguration;

import frc.robot.Constants;

public class CtreSettings {

    private static CANCoder backLeftCoder = new CANCoder(Constants.BACK_LEFT_MODULE_STEER_ENCODER);
    private static CANCoder backRightCoder = new CANCoder(Constants.BACK_RIGHT_MODULE_STEER_ENCODER);
    private static CANCoder frontLeftCoder = new CANCoder(Constants.FRONT_LEFT_MODULE_STEER_ENCODER);
    private static CANCoder frontRightCoder = new CANCoder(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER);

    public static void PrintAllCanEncoderCurrentSettings()
    {
        CtreSettings.PrintCanEncoderCurrentSettings(backLeftCoder);
        CtreSettings.PrintCanEncoderCurrentSettings(backRightCoder);
        CtreSettings.PrintCanEncoderCurrentSettings(frontLeftCoder);
        CtreSettings.PrintCanEncoderCurrentSettings(frontRightCoder);
    }

    public static void UpdateCanEncoderDefaultSettings()
    {
        CtreSettings.UpdateSingleCanEncoderDefaultSettings(backLeftCoder);
        CtreSettings.UpdateSingleCanEncoderDefaultSettings(backRightCoder);
        CtreSettings.UpdateSingleCanEncoderDefaultSettings(frontLeftCoder);
        CtreSettings.UpdateSingleCanEncoderDefaultSettings(frontRightCoder);

        PrintAllCanEncoderCurrentSettings();
    }

    public static void ZeroAllSwerveSteerMotorEncoders()
    {
        WPI_TalonFX frontLeftSteerMotor = new WPI_TalonFX(Constants.FRONT_LEFT_MODULE_STEER_MOTOR);
        WPI_TalonFX frontRightSteerMotor = new WPI_TalonFX(Constants.FRONT_RIGHT_MODULE_STEER_MOTOR);
        WPI_TalonFX backLeftSteerMotor = new WPI_TalonFX(Constants.BACK_LEFT_MODULE_STEER_MOTOR);
        WPI_TalonFX backRightSteerMotor = new WPI_TalonFX(Constants.BACK_RIGHT_MODULE_STEER_MOTOR);

        frontLeftSteerMotor.setSelectedSensorPosition(0.0);
        frontRightSteerMotor.setSelectedSensorPosition(0.0);
        backLeftSteerMotor.setSelectedSensorPosition(0.0);
        backRightSteerMotor.setSelectedSensorPosition(0.0);
    }

    private static void PrintCanEncoderCurrentSettings(CANCoder cancoder)
    {
        System.out.println("Get side, has angle: " + cancoder.getAbsolutePosition() + " and "
        + "initializationStrategy == " + cancoder.configGetSensorInitializationStrategy(0).toString());
    }
    
    private static void UpdateSingleCanEncoderDefaultSettings(CANCoder cancoder)
    {
        CANCoderConfiguration config = new CANCoderConfiguration();
        SensorInitializationStrategy strat = SensorInitializationStrategy.BootToAbsolutePosition;
        config.initializationStrategy = strat;
        ErrorCode returnVal = cancoder.configAllSettings(config);
        System.out.println("attemtpted set value == " + strat.toString());
        System.out.println("Set side, getDeviceID == " + cancoder.getDeviceID() + " Error Code: " + returnVal);
        CtreSettings.PrintCanEncoderCurrentSettings(cancoder);
    }
}
