package frc.robot;

public class DriveStraight {
    private double iterations;
    private double angle;
    private double robotAngle;
    private SwerveDrive drive;
    private int timePassed = 0;
    private boolean stop = true;
    private double speed = 0.5;

    public DriveStraight(SwerveDrive drive, double iterations, double angle, double robotAngle){
        this.iterations = iterations;
        this.angle =angle;
        this.drive = drive;
        this.robotAngle = robotAngle;
    }

    public DriveStraight(SwerveDrive drive, double iterations, double angle, double robotAngle, boolean stop){
        this.iterations = iterations;
        this.angle =angle;
        this.robotAngle = robotAngle;
        this.drive = drive;
        this.stop = stop;
    }

    public boolean driveStraight(){
        timePassed = timePassed+1;
        if(timePassed >= iterations){
            if(stop){
                drive.stopSwerveDrive();
            }
            return true;
        }else{
            drive.moveSwerveDriveAngle(speed, angle, robotAngle);
            return false;
        }
    }
}
