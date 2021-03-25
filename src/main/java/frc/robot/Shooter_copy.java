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
import edu.wpi.first.wpiutil.math.MathUtil;


public class Shooter_copy {
    TalonSRX pitchMotor;
    TalonSRX rotationMotor;
    TalonFX flywheelMotor;
    TalonSRX indexMotor;
    PIDController angleAdjuster;
    NetworkTable table;
    DigitalInput rotationLimit;
    DigitalInput pitchLimit;

    int startingPitch = 0;


    public Shooter_copy(int pitchMotorPort, int rotationMotorPort,int flywheelMotorPort, int indexMotorPort){
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
		flywheelMotor.config_kF(0, 1023.0/20660.0, 30);
		flywheelMotor.config_kP(0, 0.1, 30);
		flywheelMotor.config_kI(0, 0.001, 30);
        flywheelMotor.config_kD(0, 5, 30);
        
        angleAdjuster = new PIDController(1,0.0,0.05);

        angleAdjuster.setTolerance(10,20);
        angleAdjuster.reset();

        pitchMotor.configFactoryDefault(30);
        pitchMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);

        rotationLimit = new DigitalInput(0);
        pitchLimit = new DigitalInput(1);

        /* Factory Default all hardware to prevent unexpected behaviour */
        rotationMotor.configFactoryDefault();
		
		/* Config the sensor used for Primary PID and sensor direction */
        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
                                            0,
				                            30);

		/* Ensure sensor is positive when output is positive */
		rotationMotor.setSensorPhase(true);

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
		rotationMotor.config_kP(0, 0.15, 30);
		rotationMotor.config_kI(0, 0.0, 30);
		rotationMotor.config_kD(0, 1.0, 30);

		/**
		 * Grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match.
		 */
		
		/* Set the quadrature (relative) sensor to match absolute */
        rotationMotor.setSelectedSensorPosition(0/*absolutePosition*/, 0, 30);
        
                /* Factory Default all hardware to prevent unexpected behaviour */
        rotationMotor.configFactoryDefault();
		
		/* Config the sensor used for Primary PID and sensor direction */
        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
                                            0,
				                            30);

		/* Ensure sensor is positive when output is positive */
		rotationMotor.setSensorPhase(true);

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
		rotationMotor.config_kP(0, 0.15, 30);
		rotationMotor.config_kI(0, 0.0, 30);
		rotationMotor.config_kD(0, 1.0, 30);

		/**
		 * Grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match.
		 */
		
		/* Set the quadrature (relative) sensor to match absolute */
        rotationMotor.setSelectedSensorPosition(0/*absolutePosition*/, 0, 30);


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
		pitchMotor.setInverted(false);

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
		pitchMotor.config_kP(0, 0.15, 30);
		pitchMotor.config_kI(0, 0.0, 30);
		pitchMotor.config_kD(0, 1.0, 30);

		/**
		 * Grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match.
		 */
		
		/* Set the quadrature (relative) sensor to match absolute */
		pitchMotor.setSelectedSensorPosition(0/*absolutePosition*/, 0, 30);
    }

    //public void setFlywheelSpeed(double power){
    //    flywheelMotor.set(TalonFXControlMode.Velocity, power);
    //}

    public void flywheelSpeed(boolean power){
        if(power){
            //flywheelMotor.set(TalonFXControlMode.Velocity, 30000);
            flywheelMotor.set(ControlMode.PercentOutput, 0.40);
            //double currentSpeed = flywheelMotor.getSelectedSensorVelocity(0);
            //System.out.println(currentSpeed);
        }else{
            flywheelMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        }
    }

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

    public void moveTurret(double rotationSpeed, double pitchSpeed){
        rotationMotor.set(ControlMode.PercentOutput, rotationSpeed);
        pitchMotor.set(ControlMode.PercentOutput, pitchSpeed);
    }

    public void manuelTurret(int pov){
        double pitchSpeed;
        double rotationSpeed;

        if(pov == 315||pov == 270||pov == 225){
            rotationSpeed = -0.1;
        }else if(pov == 45||pov == 90||pov == 135){
            rotationSpeed = 0.1;
        }else{
            rotationSpeed = 0;
        }

        double pitchMag = -0.1;
        if(pov == 135||pov == 180||pov == 225){
            pitchSpeed = -1 * pitchMag;
        }else if(pov == 0||pov == 45||pov == 315){
            pitchSpeed = pitchMag;
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

        angle = angle*(1/5);
        angle = angle + startingPitch;
        

        return angle;
    }

    public void setTurretPitch(double targetPitch){
        pitchMotor.set(ControlMode.Position, targetPitch);
    }

    public void setTurretAngle(double targetAngle){
        rotationMotor.set(ControlMode.Position, targetAngle);
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
        } else {
            rotationSpeed = -1*rotationSpeed;
        }

        //pitchSpeed = setTurretPitch(y);     //This is actually going to be based off of area.  

        moveTurret(rotationSpeed, pitchSpeed);
    }

    public boolean homeTurret(){

        double pitchSpeed = 0;
        double rotationSpeed = 0;
        boolean pitchPosition = false;

        if(!pitchLimit.get()) {
            pitchSpeed = 0.5;
        } else {
            pitchSpeed = 0.0;
            //pitchMotor.resetSensorPosition();
            pitchPosition = true;
        }

        if(!rotationLimit.get()) {
            rotationSpeed = 0.5;
            moveTurret(rotationSpeed, pitchSpeed);
            return false;
        } else {
            rotationSpeed = 0.0;
            //rotationMotor.resetSensorPosition();
            moveTurret(rotationSpeed, pitchSpeed);

            if(pitchPosition){
                return true;
            }else{
                return false;
            }
        }
    }
}
