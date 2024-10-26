package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Armgrabber extends SubsystemBase {

    public CRServo leftServo;
    public CRServo rightServo;

    public Armgrabber(HardwareMap map){

        leftServo = map.get(CRServo.class, "las");
        rightServo = map.get(CRServo.class, "ras");

    }

    public void Grab(double p) {

        leftServo.setPower(p);
        rightServo.setPower(p);

    }

    public void brake() {

        leftServo.setPower(0);
        rightServo.setPower(0);

    }

}
