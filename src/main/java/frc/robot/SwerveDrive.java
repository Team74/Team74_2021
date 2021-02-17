package frc.robot;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

public class SwerveDrive {

    SwerveDriveKinematics drive;
    private SwerveModule[] module;
    public SwerveDrive(SwerveModule[] module){
        this.module = module; 
        double wheelDistance = 0.295275;
        drive = new SwerveDriveKinematics(
            new Translation2d(-1*wheelDistance, wheelDistance),          //Front Left
            new Translation2d(-1*wheelDistance, -1*wheelDistance),       //Front Right
            new Translation2d(wheelDistance, -1*wheelDistance),    //Back Right
            new Translation2d(wheelDistance, wheelDistance)        //Back Left
        );
    }

    public void MoveSwerveDrive(ChassisSpeeds speed){
        SwerveModuleState[] moduleStates = drive.toSwerveModuleStates(speed);
        for(int index = 0; index<4; index++){
            SwerveModuleState newModuleState = SwerveModuleState.optimize(moduleStates[index], new Rotation2d(module[index].getAngle()*Math.PI/180));
            module[index].SetDriveSpeed(newModuleState.speedMetersPerSecond);
            module[index].GoToAngle(newModuleState.angle.getDegrees());
        }
    }

    public double[] getAngle(){
        double[] angles = new double[4];
        for(int index = 0; index<4; index++){
            angles[index] = module[index].getAngle();
        }
        return angles;
    }
}
