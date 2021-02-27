package frc.robot;

public class DriveStraight {
    private double iterations;          //each iteration is 1/50th of a second
    private double angle;               //this is the direction the robot is DRIVING
    private double robotAngle;          //this is the direction the robot is FACING
    private SwerveDrive drive;
    private int timePassed = 0;
    private boolean stop = true;        //if this is set to false the robot will not stop it's motors at the end 
    private double speed = 0.5;         //editing this number changes the speed for all DriveStraights

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

    public boolean driveStraight(){         //call this function every 20ms
        timePassed = timePassed+1;          //timePassed is a measure of how many times you've called the function
        if(timePassed >= iterations){           
            if(stop){
                drive.stopSwerveDrive();
            }
            return true;                    //returns true if it's been longer than the number of iterations
        }else{
            drive.moveSwerveDriveAngle(speed, angle, robotAngle);
            return false;                   //returns false if not
        }
    }
}
