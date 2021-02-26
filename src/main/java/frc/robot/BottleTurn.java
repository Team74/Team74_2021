package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;

public class BottleTurn {
    double endDegree;
    boolean direction;
    SwerveDrive drive;
    NetworkTable table;

    public BottleTurn(SwerveDrive drive, double endDegree, boolean direction){
        this.endDegree = endDegree;
        this.direction = direction;
        this.drive = drive;
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean bottleTurn(){
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
    
        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        double driveControllerRightX = 0;
        double driveControllerLeftX = 0;
        double driveControllerLeftY = 0;

        if(!direction){
            if(area>2){
              if(x>10){
                driveControllerRightX = 0.2;
              }else if(x<-10){
                driveControllerRightX = 0.4;
              }else{
                driveControllerRightX = 0.3;
              }
      
              driveControllerLeftX = 0.2;

              if(area>6.5){
                driveControllerLeftY = -0.1;
              }else if(area<5.5){
                driveControllerLeftY = 0.1;
              }else{
                driveControllerLeftY = 0.0;
              }

            }else{
                driveControllerLeftY = 0.25;

                if(x>10){
                    driveControllerRightX = -0.2;
                }else if(x<-10){
                    driveControllerRightX = 0.2;
                }else{
                    driveControllerRightX = 0.0;
                }
            }
      

        }else{
            if(area>2){
                if(x<-10){
                    driveControllerRightX = -0.2;
                }else if(x>10){
                    driveControllerRightX = -0.4;
                }else{
                    driveControllerRightX = -0.3;
                }
        
                driveControllerLeftX = -0.2;

                
                if(area>6.5){
                    driveControllerLeftY = -0.1;
                }else if(area<5.5){
                    driveControllerLeftY = 0.1;
                }else{
                    driveControllerLeftY = 0.0;
                }
            }else{
                if(x>10){
                    driveControllerRightX = -0.2;
                }else if(x<-10){
                    driveControllerRightX = 0.2;
                }else{
                    driveControllerRightX = 0.0;
                }
                driveControllerLeftY = 0.25;
            }
        
        
        
        }

        ChassisSpeeds speed = new ChassisSpeeds(
            driveControllerLeftY,
            driveControllerLeftX,
            driveControllerRightX
        );

        drive.MoveSwerveDrive(speed);

        double gyro = drive.getGyro();
        if((gyro<(endDegree+5))&&(gyro>(endDegree-5))){
            drive.stopSwerveDrive();
            return true;
        }else{
            return false;
        }
    }

}
