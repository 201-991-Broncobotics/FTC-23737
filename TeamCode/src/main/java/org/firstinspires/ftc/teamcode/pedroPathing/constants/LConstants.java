package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {

        TwoWheelConstants.forwardTicksToInches = 0.001996837378676409;
        TwoWheelConstants.strafeTicksToInches =  0.0010698055860546931;
        TwoWheelConstants.forwardY = -7;
        TwoWheelConstants.strafeX = 6.2;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "fR";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "bL";
        TwoWheelConstants.forwardEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

    }
}




