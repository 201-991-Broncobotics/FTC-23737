package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {

        TwoWheelConstants.forwardTicksToInches = 0.0019848266825179;
        TwoWheelConstants.strafeTicksToInches =  0.00107103909;
        TwoWheelConstants.forwardY = -6.75;
        TwoWheelConstants.strafeX = 5.6;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "fR";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "fL";
        TwoWheelConstants.forwardEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

    }
}




