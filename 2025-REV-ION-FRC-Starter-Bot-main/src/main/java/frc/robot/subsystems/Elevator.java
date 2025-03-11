package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PidController;



public class Elevator extends SubsystemBase {
    
    private final SparkFlex elevatorMotor;

    private final double elevatorSpeed = 0.3;

    private final PidController pidController;



    
    public Elevator() {
        elevatorMotor  = new SparkFlex(12, SparkLowLevel.MotorType.kBrushless);
        pidController = new PidController(0.3, 0.1, 0, elevatorMotor);
    }

    public void setMotors(double percent) {
        
        elevatorMotor.set(-percent * elevatorSpeed);
        

        SmartDashboard.putNumber("Elevator Power(%)", percent * elevatorSpeed);
        SmartDashboard.putNumber("Elevator Pos", elevatorMotor.getEncoder().getPosition());

        if(percent == 0) {
            elevatorMotor.stopMotor();
        }
    }

    public void setTarget(double target) {
        //pidController.setTargetPoint(target);
        
    }

    
    

    public void update() {
       // pidController.update();

        //setMotors(pidController.GetForce());


    }

    public Command GetTeleopCommand(XboxController controller) {
        return run(() -> {
            setMotors(MathUtil.applyDeadband(controller.getRightY(), 0.1));
        });
    }


    public static Command GetAutoCommand(Elevator elevator, double target) {
        return new Command() {
            

            @Override
            public void initialize() {

                elevator.pidController.setTargetPoint(target);
            }

            @Override
            public void execute() {
                elevator.update();
            }

            @Override
            public boolean isFinished() {
                return elevator.pidController.IsAtTarget();
            }
        };
    }
}