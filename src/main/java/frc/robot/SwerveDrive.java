package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpiutil.math.MathUtil;

public class SwerveDrive {

    SwerveDriveKinematics drive;
    private SwerveModule[] module;
    private AHRS gyro;
    PIDController angleAdjuster;
    PIDController robotAngle;

    public SwerveDrive(SwerveModule[] module, AHRS gyro){
        this.module = module; 
        double wheelDistance = 0.295275;
        drive = new SwerveDriveKinematics(
            new Translation2d(-1*wheelDistance, wheelDistance),          //Front Left
            new Translation2d(-1*wheelDistance, -1*wheelDistance),       //Front Right
            new Translation2d(wheelDistance, -1*wheelDistance),    //Back Right
            new Translation2d(wheelDistance, wheelDistance)        //Back Left
        );
        this.gyro = gyro;
        angleAdjuster = new PIDController(1,0.0,0.05);
        angleAdjuster.enableContinuousInput(-180,180);
        angleAdjuster.setTolerance(20,40);
        angleAdjuster.reset();
        robotAngle = new PIDController(1,0.0,0.20);
        robotAngle.setTolerance(20,40);
        robotAngle.reset();
    }

    public void MoveSwerveDrive(ChassisSpeeds speed){
        SwerveModuleState[] moduleStates = drive.toSwerveModuleStates(speed);
        for(int index = 0; index<4; index++){
            SwerveModuleState newModuleState = SwerveModuleState.optimize(moduleStates[index], new Rotation2d(module[index].getAngle()*Math.PI/180));
            module[index].SetDriveSpeed(newModuleState.speedMetersPerSecond);
            module[index].GoToAngle(newModuleState.angle.getDegrees());
        }
    }

    public void moveSwerveDriveAbsoluteAngle(double speed, double angle){
        double gyroAngle = normalizeAngle(gyro.getAngle());
        double rotation = robotAngle.calculate(gyroAngle,angle);
        rotation = rotation/180;
        rotation = MathUtil.clamp(rotation,-1.0,1.0);
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            speed,
            0,
            rotation,
            new Rotation2d((gyro.getAngle()-angle)*Math.PI/180)
        );
        MoveSwerveDrive(speeds);
    }

    public void moveSwerveDriveAngle(double speed, double angle, double robotAngle){
        double gyroAngle = normalizeAngle(gyro.getAngle());
        double rotation = this.robotAngle.calculate(gyroAngle,robotAngle);
        rotation = rotation/180;
        rotation = MathUtil.clamp(rotation,-1.0,1.0);
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            speed,
            0,
            rotation,
            new Rotation2d((gyro.getAngle()-angle)*Math.PI/180)
        );
        MoveSwerveDrive(speeds);
    }

    public void stopSwerveDrive(){
        ChassisSpeeds speeds = new ChassisSpeeds(
            0,
            0,
            0
        );
        MoveSwerveDrive(speeds);
    }

    public double[] getAngle(){
        double[] angles = new double[4];
        for(int index = 0; index<4; index++){
            angles[index] = module[index].getAngle();
        }
        return angles;
    }

    private double normalizeAngle(double angle) {return ((angle%360)+360)%360-180;}

	public void MoveSwerveDrive(double d, double angle) {
	}
}
