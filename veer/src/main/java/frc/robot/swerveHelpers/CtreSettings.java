package frc.robot.swerveHelpers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.swervedrivespecialties.swervelib.ctre.CanCoderAbsoluteConfiguration;

import frc.robot.Constants;

public class CtreSettings {

    public static void PrintAllCanEncoderCurrentSettings()
    {
        CANCoder backLeftCoder = new CANCoder(Constants.BACK_LEFT_MODULE_STEER_ENCODER);
        CANCoder backRightCoder = new CANCoder(Constants.BACK_RIGHT_MODULE_STEER_ENCODER);
        CANCoder frontLeftCoder = new CANCoder(Constants.FRONT_LEFT_MODULE_STEER_ENCODER);
        CANCoder frontRightCoder = new CANCoder(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER);

        CtreSettings.PrintCanEncoderCurrentSettings(backLeftCoder);
        CtreSettings.PrintCanEncoderCurrentSettings(backRightCoder);
        CtreSettings.PrintCanEncoderCurrentSettings(frontLeftCoder);
        CtreSettings.PrintCanEncoderCurrentSettings(frontRightCoder);
    }

    public static void UpdateCanEncoderDefaultSettings()
    {
        CANCoder backLeftCoder = new CANCoder(Constants.BACK_LEFT_MODULE_STEER_ENCODER);
        CANCoder backRightCoder = new CANCoder(Constants.BACK_RIGHT_MODULE_STEER_ENCODER);
        CANCoder frontLeftCoder = new CANCoder(Constants.FRONT_LEFT_MODULE_STEER_ENCODER);
        CANCoder frontRightCoder = new CANCoder(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER);

        CtreSettings.UpdateSingleCanEncoderDefaultSettings(backLeftCoder);
        CtreSettings.UpdateSingleCanEncoderDefaultSettings(backRightCoder);
        CtreSettings.UpdateSingleCanEncoderDefaultSettings(frontLeftCoder);
        CtreSettings.UpdateSingleCanEncoderDefaultSettings(frontRightCoder);
    }

    private static void PrintCanEncoderCurrentSettings(CANCoder cancoder)
    {
        System.out.println("Get side, has initializationStrategy == " + cancoder.configGetSensorInitializationStrategy(0).toString());
    }
    
    private static void UpdateSingleCanEncoderDefaultSettings(CANCoder cancoder)
    {
        CANCoderConfiguration config = new CANCoderConfiguration();
        SensorInitializationStrategy strat = SensorInitializationStrategy.BootToAbsolutePosition;
        config.initializationStrategy = strat;
        ErrorCode returnVal = cancoder.configAllSettings(config);
        System.out.println("Set side, getDeviceID == " + cancoder.getDeviceID() + " Error Code: " + returnVal);
    }
}
