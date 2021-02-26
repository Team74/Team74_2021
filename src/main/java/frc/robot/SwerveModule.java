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
        angleAdjuster = new PIDController(1,0.0,0.05);
        angleAdjuster.enableContinuousInput(-180,180);
        angleAdjuster.setTolerance(10,20);
        angleAdjuster.reset();
    }

    //public void SetPIDParameters(double p, double d){
    //    angleAdjuster.setPID(p,0,d);
    //}

    public void GoToAngle(double desiredAngle){
        double testDouble = angleAdjuster.calculate(rotationEncoder.getAngle(),desiredAngle);
        //testDouble = angleAdjuster.getPositionError();
        //SmartDashboard.putNumber("PID Value", testDouble);
        //SmartDashboard.putNumber("Error Value", angleAdjuster.getPositionError());
        testDouble = testDouble/180;
        testDouble = MathUtil.clamp(testDouble,-1.0,1.0);
        //SmartDashboard.putNumber("Current Angle", rotationEncoder.getAngle());
        if(angleAdjuster.atSetpoint()){
            //System.out.println("At Set Angle");
            rotationMotor.set(0);
        } else{
            //System.out.println(testDouble);
            rotationMotor.set(-1*testDouble);
        }
        
    }

    public void SetDriveSpeed(double speed){
        speed = MathUtil.clamp(speed,-1.0,1.0);
        driveMotor.set(-1*speed);
    }

    public void StopRotation(){
        rotationMotor.set(0);
        SmartDashboard.putNumber("Current Angle", rotationEncoder.getAngle());
    }

    public void stopMotor(){
        rotationMotor.set(0);
        driveMotor.set(0);
    }

    public double getAngle(){
        return rotationEncoder.getAngle();
    }
}
