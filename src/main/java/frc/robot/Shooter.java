package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpiutil.math.MathUtil;


public class Shooter {
    TalonSRX pitchMotor;
    TalonSRX rotationMotor;
    TalonFX flywheelMotor;
    VictorSPX indexMotor;
    PIDController angleAdjuster;
    NetworkTable table;
    int startingPitch = 0;

    public Shooter(int pitchMotorPort, int rotationMotorPort,int flywheelMotorPort, int indexMotorPort){
        table = NetworkTableInstance.getDefault().getTable("limelight");
        pitchMotor = new TalonSRX(pitchMotorPort);
        rotationMotor = new TalonSRX(rotationMotorPort);
        indexMotor = new VictorSPX(indexMotorPort);
        flywheelMotor = new TalonFX(flywheelMotorPort);
                /* Factory Default all hardware to prevent unexpected behaviour */
		flywheelMotor.configFactoryDefault();
		
		/* Config neutral deadband to be the smallest possible */
		flywheelMotor.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
        flywheelMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
											

		/* Config the peak and nominal outputs */
		flywheelMotor.configNominalOutputForward(0, 30);
		flywheelMotor.configNominalOutputReverse(0, 30);
		flywheelMotor.configPeakOutputForward(1, 30);
		flywheelMotor.configPeakOutputReverse(-1, 30);

		/* Config the Velocity closed loop gains in slot0 */
		flywheelMotor.config_kF(0, 1023.0/20660.0, 30);
		flywheelMotor.config_kP(0, 0.1, 30);
		flywheelMotor.config_kI(0, 0.001, 30);
        flywheelMotor.config_kD(0, 5, 30);
        
        angleAdjuster = new PIDController(1,0.0,0.05);

        angleAdjuster.setTolerance(10,20);
        angleAdjuster.reset();

        pitchMotor.configFactoryDefault(30);
        pitchMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
    }

    public void setFlywheelSpeed(double power){
        flywheelMotor.set(TalonFXControlMode.Velocity, power);
    }

    public void flywheelSpeed(boolean power){
        if(power){
			flywheelMotor.set(TalonFXControlMode.Velocity, 10000);
        }else{
            flywheelMotor.set(TalonFXControlMode.Velocity, 0.0);
        }
    }

    public void activateShooter(){
        indexMotor.set(ControlMode.PercentOutput, 1.0);
    }

    public boolean isFlywheelUpToSpeed(int targetSpeed){
        double currentSpeed = flywheelMotor.getSelectedSensorVelocity(0);
        return (currentSpeed >= (targetSpeed - 1000) && currentSpeed <= (targetSpeed + 1000));
    }

    public void moveTurret(double rotationSpeed, double pitchSpeed){
        rotationMotor.set(ControlMode.PercentOutput, rotationSpeed);
        pitchMotor.set(ControlMode.PercentOutput, pitchSpeed);
    }

    public void manuelTurret(int pov){
        double pitchSpeed;
        double rotationSpeed;

        if(pov == 315||pov == 270||pov == 225){
            rotationSpeed = -1;
        }else if(pov == 45||pov == 90||pov == 135){
            rotationSpeed = 1;
        }else{
            rotationSpeed = 0;
        }

        if(pov == 135||pov == 180||pov == 225){
            pitchSpeed = -1;
        }else if(pov == 0||pov == 45||pov == 315){
            pitchSpeed = 1;
        }else{
            pitchSpeed = 0;
        }

        moveTurret(rotationSpeed, pitchSpeed);
    }

    public double findPitch(){
        double pitch = 0;

        pitch = pitchMotor.getSelectedSensorPosition();

        pitch = pitch*(1/5);
        pitch = pitch + startingPitch;
        

        return pitch;
    }

    public double setTurretPitch(double targetPitch){
        double pitchSpeed = 0;
        pitchSpeed = angleAdjuster.calculate(targetPitch, findPitch());
        return pitchSpeed;
    }

    public void autoTurret(){
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        double pitchSpeed = 0;
        double rotationSpeed;

        rotationSpeed = angleAdjuster.calculate(0, x);

        rotationSpeed = rotationSpeed/180;
        rotationSpeed = MathUtil.clamp(rotationSpeed,-1.0,1.0);

        if(angleAdjuster.atSetpoint()){

            rotationSpeed = 0;
        } else{

            rotationSpeed = 0;
        }

        pitchSpeed = setTurretPitch(y);

        moveTurret(rotationSpeed, pitchSpeed);
    }
}
