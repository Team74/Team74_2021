package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class Ma3Encoder {
    int portNumber;
    double offsetAngle;

    AnalogInput encoder;
    public Ma3Encoder(int portNumber, double offsetAngle){
        this.portNumber = portNumber;
        this.offsetAngle = offsetAngle;
        encoder = new AnalogInput(portNumber);
        encoder.setAverageBits(4);
    }

    public double getAngle(){
        int value = encoder.getAverageValue();
        double Angle = value*360.0/4096;
        Angle = (Angle-offsetAngle+360)%360-180; 
        return Angle;
    }
}
