package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        angleAdjuster = new PIDController(0.00005,0.0,0.0005);
        angleAdjuster.enableContinuousInput(-180,180);
        angleAdjuster.setTolerance(20,40);
        angleAdjuster.reset();
    }

    public void SetPIDParameters(double p, double d){
        angleAdjuster.setPID(p,0,d);
    }

    public void GoToAngle(double desiredAngle){
        double testDouble = angleAdjuster.calculate(rotationEncoder.getAngle(),desiredAngle);
        //testDouble = angleAdjuster.getPositionError();
        SmartDashboard.putNumber("PID Value", testDouble);
        SmartDashboard.putNumber("Error Value", angleAdjuster.getPositionError());
        testDouble = testDouble/180;
        testDouble = MathUtil.clamp(testDouble,-0.2,0.2);
        SmartDashboard.putNumber("Current Angle", rotationEncoder.getAngle());
        if(angleAdjuster.atSetpoint()){
            //System.out.println("At Set Angle");
            rotationMotor.set(0);
        } else{
            //System.out.println(testDouble);
            rotationMotor.set(-1*testDouble);
        }
        
    }

    public void StopRotation(){
        rotationMotor.set(0);
        SmartDashboard.putNumber("Current Angle", rotationEncoder.getAngle());
    }
}
