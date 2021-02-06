package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpiutil.math.MathUtil;

public class SwerveModule {
    CANSparkMax rotationMotor;
    CANSparkMax driveMotor;
    Ma3Encoder rotationEncoder;
    PIDController angleAdjuster;

    public SwerveModule(int driveMotorPort, int rotationMotorPort,int rotationEncoderPort, double rotationEncoderOffsetAngle){
        rotationMotor = new CANSparkMax(rotationMotorPort,MotorType.kBrushless);
        driveMotor = new CANSparkMax(driveMotorPort,MotorType.kBrushless);
        rotationEncoder = new Ma3Encoder(rotationEncoderPort,rotationEncoderOffsetAngle);
        angleAdjuster = new PIDController(0.005,0.0,0);
        angleAdjuster.enableContinuousInput(-180,180);
        angleAdjuster.setTolerance(20,40);
        angleAdjuster.reset();
    }

    public void GoToAngle(double desiredAngle){
        double testDouble = angleAdjuster.calculate(rotationEncoder.getAngle(),desiredAngle);
        //testDouble = angleAdjuster.getPositionError();
        testDouble = testDouble/18;
        testDouble = MathUtil.clamp(testDouble,-0.1,0.1);
        if(angleAdjuster.atSetpoint()){
            System.out.println("At Set Angle");
        } else{
            System.out.println(testDouble);
            rotationMotor.set(testDouble);
        }
        
    }
}
