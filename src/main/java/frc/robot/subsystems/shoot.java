package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shoot extends SubsystemBase{
     private Timer AutoTimer2 = new Timer();
    private final TalonFX shootMotor = new TalonFX(21);

    public shoot(){

    }
    @Override
    public void periodic(){
        // This method will be called once per scheduler run
    }

    public Command shootcmd(double speed){
    
        
    return new Command() {
        @Override
        public void initialize() {
            // Initialization code, such as resetting encoders or PID controllers

        }

        @Override
        public void execute() {
            shootMotor.set(speed);
          
        }

        @Override
        public void end(boolean interrupted) {
        
          
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    };}

    public Command Autoshootcmd(double speed){
    
        
        return new Command() {
            @Override
            public void initialize() {
                AutoTimer2.reset();
                AutoTimer2.start();
                // Initialization code, such as resetting encoders or PID controllers
    
            }
    
            @Override
            public void execute() {
                shootMotor.set(speed);
              
            }
    
            @Override
            public void end(boolean interrupted) {
                shootMotor.set(0);
              
            }
    
            @Override
            public boolean isFinished() {
                return AutoTimer2.get() > 1;
            }
        };
    }
   
    
}