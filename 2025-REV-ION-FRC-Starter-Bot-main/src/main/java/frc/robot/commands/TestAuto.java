package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TestAuto extends Command {
 
    double autonomousStartTime = 0;
    DriveSubsystem m_robotDrive;
    public TestAuto(DriveSubsystem m_robotDrive) {
        addRequirements(m_robotDrive);
        this.m_robotDrive = m_robotDrive;
        
    }
    @Override
    public void initialize() {
        // Runs once on start  
        autonomousStartTime = Timer.getFPGATimestamp();
 
    }

    @Override
    public void execute() {
        //System.out.println("Works");
        SmartDashboard.putNumber("Timestamp", Timer.getFPGATimestamp());
        
        double elapsedTime = Timer.getFPGATimestamp() - autonomousStartTime;
        // Runs repeatedly after initialization
        if (elapsedTime < 1.0) {
            //new runLauncher(m_launcher, () -> 1);
            m_robotDrive.drive(
                0.50,
                0,
                0,
                true,
                false
                ); 
        }
        else if (elapsedTime < 2.0){
            //Next item to implement If it is to wait write code to make it "Stop" or
            //Zero out the Drive train.
                m_robotDrive.drive(0,
                    0,
                    0,
                    true,
                    false);
        }
         else {
           
        }
    }


    @Override
    public void end(boolean interrupted) {
        // Runs when ended/cancelled
    }

    @Override
    public boolean isFinished() {
        return false;
        // Whether or not the command is finished
    }

}
