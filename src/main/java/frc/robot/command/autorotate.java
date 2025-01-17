package frc.robot.command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.opencv.photo.Photo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constant;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.function.Supplier;


import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.subsystems.vision;
import frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

public class autorotate extends Command {
    private vision s_vision;
    private CommandSwerveDrivetrain s_Swerve; 

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup1;
    private BooleanSupplier robotCentricSup;
    private DoubleSupplier ang;
    private double prevdistance = 100;
    private double setpoint =  15;
  private  CommandSwerveDrivetrain controller;

   public autorotate(vision photonvision ,CommandSwerveDrivetrain s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup1) {
        s_vision = photonvision;
        this.s_Swerve = s_Swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup1 = rotationSup1;
        this.robotCentricSup = robotCentricSup;

        // this.controller = s_Swerve.getSwerveController();
   }

   @Override
   public void execute(){
    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), 0.2);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), 0.2);

        double rotationSup = MathUtil.applyDeadband(strafeSup.getAsDouble(), 0.2);
        // double distFactor = 0.2 * (s_vision.getRZ() > 5 ? 5 : s_vision.getRZ());
    //     /* Get rotation */
        PIDController turnController = new PIDController(0.04, 0.0001, 0.000005);
        turnController.enableContinuousInput(-180, 180);

   

        var result = s_vision.getLatestResult();
        
        if (result.hasTargets()) {
            double rotationspeed = -turnController.calculate(result.getBestTarget().getYaw(),0);
            new Drive(s_Swerve,translationSup,strafeSup,()->rotationspeed);

            
            
             
        } else {
            new Drive(s_Swerve, strafeSup, strafeSup, rotationSup1);
        }
   }

    
}
