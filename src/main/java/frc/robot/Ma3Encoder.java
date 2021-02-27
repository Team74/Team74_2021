package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class Ma3Encoder {

    double offsetAngle;                                  
    AnalogInput encoder;

    public Ma3Encoder(int portNumber, double offsetAngle){
        this.offsetAngle = offsetAngle;                  
        encoder = new AnalogInput(portNumber);           //Creates the encoder

        encoder.setAverageBits(4);                       //I think this means it takes 
    }                                                    //the average of 4 encoder readings

    public double getAngle(){
        int encoderValue = encoder.getAverageValue();    //encoderValue is on a range from 0 to 4096
        double Angle = encoderValue*360.0/4096;          //converts encoderValue to degrees
        Angle = Angle-offsetAngle;                       //Subtracts the offsetAngle from the Angle
        Angle = (Angle+360)%360-180;                     //Bring angles into a range [-180,180)
        return Angle;                                    //Returns angle
    }
}
