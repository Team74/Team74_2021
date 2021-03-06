package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonSlalom extends Auton{

    //Initcializes the Auton Steps
    private CurvedDrive autonStep1;
    private BottleTurn autonStep2;
    private BottleTurn autonStep3;
    private CurvedDrive autonStep4;
    private BottleTurn autonStep5;
    private CurvedDrive autonStep6;
    private int autonIndex = 1; 

    public AutonSlalom(SwerveDrive drive){
        super(drive);

        //Creates the Auton Steps
        autonIndex = 1;
        autonStep1 = new CurvedDrive(drive, 3, new double[][] {{16.0,0.0,30.0},{55.0,-60.0,30.0},{90.0,0.0,35.0}});
        autonStep2 = new BottleTurn(drive, 90, false);
        autonStep3 = new BottleTurn(drive, 0, false);
        autonStep4 = new CurvedDrive(drive, 2, new double[][] {{18.0,135.0,-45.0},{110.0,-180.0,-90.0}});
        autonStep5 = new BottleTurn(drive, 0, true);
        autonStep6 = new CurvedDrive(drive, 2, new double[][] {{20.0,-135.0,45.0},{25.0,-180.0,45.0}});
    }

    public void run(){
        //SmartDashboard.putNumber("Auton Index", autonIndex);

        //Runs the current Auton Step
        if(autonIndex == 1){
          if(autonStep1.curvedDrive()){
            autonIndex++;
          }
        }else if(autonIndex == 2){
          if(autonStep2.bottleTurn()){
            autonIndex++;
          }
        }else if(autonIndex == 3){
          if(autonStep3.bottleTurn()){
            autonIndex++;
          }
        }else if(autonIndex == 4){
          if(autonStep4.curvedDrive()){
            autonIndex++;
          }
        }else if(autonIndex == 5){
          if(autonStep5.bottleTurn()){
            autonIndex++;
          }
        }else if(autonIndex == 6){
          if(autonStep6.curvedDrive()){
            autonIndex++;
          }
        }else{
          super.drive.stopSwerveDrive();
        }
    }
}
