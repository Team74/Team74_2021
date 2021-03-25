package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;


public class Shooter {
    TalonSRX pitchMotor;
    TalonSRX rotationMotor;
    TalonFX flywheelMotor;
    TalonSRX indexMotor;
    PIDController angleAdjuster;
    NetworkTable table;
    DigitalInput rotationLimit;
    DigitalInput pitchLimit;
    double startingAngle = 47.5;
    double startingPitch = 0;
    double flywheelPower = 0;
    double desiredFlywheelSpeed = 0;


    public Shooter(int pitchMotorPort, int rotationMotorPort,int flywheelMotorPort, int indexMotorPort){
        table = NetworkTableInstance.getDefault().getTable("limelight");
        pitchMotor = new TalonSRX(pitchMotorPort);
        rotationMotor = new TalonSRX(rotationMotorPort);
        indexMotor = new TalonSRX(indexMotorPort);
        flywheelMotor = new TalonFX(flywheelMotorPort);
                /* Factory Default all hardware to prevent unexpected behaviour */

        flywheelMotor.configFactoryDefault();
        flywheelMotor.setNeutralMode(NeutralMode.Brake);
		
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
		flywheelMotor.config_kF(0, 0.05, 30);
		flywheelMotor.config_kP(0, 0.5, 30);
		flywheelMotor.config_kI(0, 0.0, 30);
        flywheelMotor.config_kD(0, 0.0, 30);

        flywheelMotor.configClosedloopRamp(2.5);
        
        angleAdjuster = new PIDController(1,0.0,0.05);

        angleAdjuster.setTolerance(10,20);
        angleAdjuster.reset();

        pitchMotor.configFactoryDefault(30);
        pitchMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);

        rotationLimit = new DigitalInput(1);
        pitchLimit = new DigitalInput(0);

        /* Factory Default all hardware to prevent unexpected behaviour */
        rotationMotor.configFactoryDefault();
		
        /* Config the sensor used for Primary PID and sensor direction */
        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
                                                    0,
                                                    30);
        
        /* Ensure sensor is positive when output is positive */
        rotationMotor.setSensorPhase(false);
        
        /**
         * Set based on what direction you want forward/positive to be.
         * This does not affect sensor phase. 
         */ 
        rotationMotor.setInverted(false);
        
        /* Config the peak and nominal outputs, 12V means full */
        rotationMotor.configNominalOutputForward(0, 30);
        rotationMotor.configNominalOutputReverse(0, 30);
        rotationMotor.configPeakOutputForward(1, 30);
        rotationMotor.configPeakOutputReverse(-1, 30);
        
        /**
         * Config the allowable closed-loop error, Closed-Loop output will be
         * neutral within this range. See Table in Section 17.2.1 for native
         * units per rotation.
         */
        rotationMotor.configAllowableClosedloopError(0, 0, 30);
        
        /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
        rotationMotor.config_kF(0, 0.0, 30);
        rotationMotor.config_kP(0, 0.60, 30);
        rotationMotor.config_kI(0, 0.0, 30);
        rotationMotor.config_kD(0, 0.0, 30);
        
        /**
         * Grab the 360 degree position of the MagEncoder's absolute
         * position, and intitally set the relative sensor to match.
         */

                 /* Factory Default all hardware to prevent unexpected behaviour */
        pitchMotor.configFactoryDefault();
		
        /* Config the sensor used for Primary PID and sensor direction */
        pitchMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
                                                    0,
                                                    30);
        
        /* Ensure sensor is positive when output is positive */
        pitchMotor.setSensorPhase(true);
        
        /**
         * Set based on what direction you want forward/positive to be.
         * This does not affect sensor phase. 
         */ 
        pitchMotor.setInverted(true);
        
        /* Config the peak and nominal outputs, 12V means full */
        pitchMotor.configNominalOutputForward(0, 30);
        pitchMotor.configNominalOutputReverse(0, 30);
        pitchMotor.configPeakOutputForward(1, 30);
        pitchMotor.configPeakOutputReverse(-1, 30);
        
        /**
         * Config the allowable closed-loop error, Closed-Loop output will be
         * neutral within this range. See Table in Section 17.2.1 for native
         * units per rotation.
         */
        pitchMotor.configAllowableClosedloopError(0, 0, 30);
        
        /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
        pitchMotor.config_kF(0, 0.0, 30);
        pitchMotor.config_kP(0, 4.0, 30);
        pitchMotor.config_kI(0, 0.0, 30);
        pitchMotor.config_kD(0, 0.0, 30);
        
        /**
         * Grab the 360 degree position of the MagEncoder's absolute
         * position, and intitally set the relative sensor to match.
         */
    }

    //public void setFlywheelSpeed(double power){
    //    flywheelMotor.set(TalonFXControlMode.Velocity, power);
    //}

    public void flywheelSpeed(boolean power){
        SmartDashboard.putNumber("Desired Flywhhel Speed", desiredFlywheelSpeed);
        if(power){
            //flywheelMotor.set(TalonFXControlMode.Velocity, 30000);
            flywheelMotor.set(TalonFXControlMode.Velocity, desiredFlywheelSpeed);
            //double currentSpeed = flywheelMotor.getSelectedSensorVelocity(0);
            //System.out.println(currentSpeed);
        }else{
            flywheelMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        }
    }

    /*public void rampFlywheel(boolean power, double speed){
        if(!power){
            flywheelPower = 0;
        }else if(flywheelPower < speed){
            flywheelPower = flywheelPower + 0.005;
        }
        SmartDashboard.putNumber("Flywheel Output", flywheelPower);
        flywheelMotor.set(TalonFXControlMode.PercentOutput, flywheelPower);
    }*/

    public void activateShooter(){
        indexMotor.set(ControlMode.PercentOutput, 1.0);
    }

    public void stopShooter(){
        indexMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public boolean isFlywheelUpToSpeed(int targetSpeed){
        double currentSpeed = flywheelMotor.getSelectedSensorVelocity(0);
        return (currentSpeed >= (targetSpeed - 500) && currentSpeed <= (targetSpeed + 500));
    }

    public double getFlywheelSpeed(){
        return flywheelMotor.getSelectedSensorVelocity(0);
    }

    public void moveTurret(double rotationSpeed, double pitchSpeed){
        SmartDashboard.putBoolean("Turret Limit", pitchLimit.get() || rotationLimit.get());
        rotationMotor.set(ControlMode.PercentOutput, rotationSpeed);
        pitchMotor.set(ControlMode.PercentOutput, pitchSpeed);
    }

    public void manuelTurret(int pov, boolean limitOveride){
        double pitchSpeed;
        double rotationSpeed;

        SmartDashboard.putBoolean("Auto Turret", false);

        //desiredFlywheelSpeed = 16000;
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);


        if(area>1.0){
            desiredFlywheelSpeed = 7900;
        }else if(area>0.46){
            desiredFlywheelSpeed = 13000;
        }else if(area>0.20){
            desiredFlywheelSpeed = 13000;
        }else{
            desiredFlywheelSpeed = 16000;
        }

        if(pov == 315||pov == 270||pov == 225){
            rotationSpeed = -0.3;
        }else if((pov == 45||pov == 90||pov == 135) && (!rotationLimit.get() || limitOveride)){
            rotationSpeed = 0.3;
        }else{
            rotationSpeed = 0;
        }

        double pitchMag = 0.3;
        if((pov == 135||pov == 180||pov == 225)){
            pitchSpeed = 1 * pitchMag;
        }else if((pov == 0||pov == 45||pov == 315) && (!pitchLimit.get() || limitOveride)){
            pitchSpeed = -1 * pitchMag;
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

    public double findAngle(){
        double angle = 0;

        angle = rotationMotor.getSelectedSensorPosition();

        angle = angle/100.0;
        angle = angle + startingAngle;
        

        return angle;
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

        double desiredRotationAngle = x;
        double desiredPitchTick = 1700;

        desiredRotationAngle = MathUtil.clamp(desiredRotationAngle, -47.5, 47.5);
        desiredRotationAngle = getRotationTick(desiredRotationAngle);

        if(area>1.0){
            desiredPitchTick = 2800;
            desiredFlywheelSpeed = 7900;
            desiredRotationAngle = desiredRotationAngle + 50;
        }else if(area>0.46){
            desiredPitchTick = 1800;
            desiredFlywheelSpeed = 13000;
        }else if(area>0.20){
            desiredPitchTick = 1830;
            desiredFlywheelSpeed = 13000;
        }else{
            desiredPitchTick = 1720;
            desiredFlywheelSpeed = 16000;
        }
        
        SmartDashboard.putNumber("Desired Pitch", desiredPitchTick);

        if(!rotationLimit.get()){
            rotationMotor.set(ControlMode.Position, desiredRotationAngle);
        }else{
            rotationMotor.set(ControlMode.PercentOutput, 0);
        }
        if(!pitchLimit.get()){
            if(pitchMotor.getSelectedSensorPosition()<3400){
                pitchMotor.set(ControlMode.Position, desiredPitchTick);
            }
        }else{
            pitchMotor.set(ControlMode.PercentOutput, 0);
        }
        SmartDashboard.putBoolean("Auto Turret", true);
        //pitchSpeed = setTurretPitch(y);     //This is actually going to be based off of area.  

        //moveTurret(rotationSpeed, pitchSpeed);
    }

    public boolean homeTurret(){

        double pitchSpeed = 0;
        double rotationSpeed = 0;
        boolean pitchPosition = false;

        SmartDashboard.putBoolean("Turret Limit", pitchLimit.get() || rotationLimit.get());

        if(!pitchLimit.get()) {
            pitchSpeed = -0.1;
        } else {
            pitchSpeed = 0.0;
            pitchMotor.setSelectedSensorPosition(0, 0, 30);
            pitchPosition = true;
        }

        if(!rotationLimit.get()) {
            rotationSpeed = 0.1;
            moveTurret(rotationSpeed, pitchSpeed);
            return false;
        } else {
            rotationSpeed = 0.0;
            rotationMotor.setSelectedSensorPosition(0, 0, 30);
            moveTurret(rotationSpeed, pitchSpeed);

            if(pitchPosition){
                return true;
            }else{
                return false;
            }
        }
    }

    public double[] getTurretPosition(){
        double[] turretPosition = {0, 0};
        turretPosition[0] = pitchMotor.getSelectedSensorPosition();
        turretPosition[1] = rotationMotor.getSelectedSensorPosition();
        return turretPosition;
    }

    public double getRotationTick(double rotationAngle){
        rotationAngle = rotationAngle - 51.0;
        rotationAngle = rotationAngle * 100;
        return rotationAngle;
    }
}
