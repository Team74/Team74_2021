package frc.robot;

public class CurvedDrive {
    SwerveDrive drive;
    int steps;
    double path[][];
    int index = 0;
    DriveStraight currentStep;
    //initialize the array

    public CurvedDrive(SwerveDrive drive, int steps, double path[][]){
        this.drive = drive;
        this.steps = steps;
        this.path = path;
        //set initilized array to the array you passed in
        currentStep = new DriveStraight(drive, path[index][0], path[index][1], false);
    }

    public boolean curvedDrive(){
        if (currentStep.driveStraight()){
            index = index+1;
            if (index >= steps){
                drive.stopSwerveDrive();
                return true;
            }else{
                currentStep = new DriveStraight(drive, path[index][0], path[index][1], false);
                //currentStep.driveStraight();
            }
        }
        return false;
    }
}
