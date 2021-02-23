package frc.robot;

public class DriveStraight {
    private double iterations;
    private double angle;
    private SwerveDrive drive;
    private int timePassed = 0;
    private boolean stop = true;
    private double speed = 0.2;

    public DriveStraight(SwerveDrive drive, double iterations, double angle){
        this.iterations = iterations;
        this.angle =angle;
        this.drive = drive;
    }

    public DriveStraight(SwerveDrive drive, double iterations, double angle, boolean stop){
        this.iterations = iterations;
        this.angle =angle;
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
            drive.moveSwerveDriveAbsoluteAngle(speed, angle);
            return false;
        }
    }
}
