package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SlideSuperStucture;
import org.firstinspires.ftc.teamcode.subsystems.AlignVision;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class FineAimCommand extends CommandBase {
    private final MecanumDrive drive;
    private final SlideSuperStucture slide;
    private final AlignVision vision;
    
    // Interpolating maps for X and Y adjustments based on vision values
    private final InterpolatingTreeMap<Double, Double> xAdjustments;
    private final InterpolatingTreeMap<Double, Double> yAdjustments;
    
    //private static final double MAX_SPEED = 0.3;
    private static final double TOLERANCE = 0.05;
    
    public FineAimCommand(MecanumDrive drive, SlideSuperStucture slide, AlignVision vision) {
        this.drive = drive;
        this.slide = slide;
        this.vision = vision;
        
        // Initialize interpolating maps
        xAdjustments = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
        yAdjustments = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
        
//        // Populate maps with calibrated values
//        // These values should be tuned based on testing
//        // Format: vision reading -> required movement
//        xAdjustments.put(-1.0, -0.3);  // Far left -> move right
//        xAdjustments.put(-0.5, -0.15);
//        xAdjustments.put(0.0, 0.0);    // Centered
//        xAdjustments.put(0.5, 0.15);
//        xAdjustments.put(1.0, 0.3);    // Far right -> move left
//
//        yAdjustments.put(-1.0, -0.3);  // Too close -> move back
//        yAdjustments.put(-0.5, -0.15);
//        yAdjustments.put(0.0, 0.0);    // Optimal distance
//        yAdjustments.put(0.5, 0.15);
//        yAdjustments.put(1.0, 0.3);    // Too far -> move forward
        
        //addRequirements(drive, slide);
    }
    
    @Override
    public void initialize() {
        // Ensure slide is in aim position
        slide.aimCommand().schedule();
        double[] visionValues = vision.getFineSearchVal();
        
        // Extract X and Y error values from vision
        double xError = visionValues[0];  // Horizontal offset
        double yError = visionValues[1];  // Distance error
        
        // Calculate movement adjustments using interpolating maps
        double xMove = xAdjustments.get(xError);
        double yMove = yAdjustments.get(yError);
        
        // Apply speed limits
        //xMove = Math.min(Math.max(xMove, -MAX_SPEED), MAX_SPEED);
        //yMove = Math.min(Math.max(yMove, -MAX_SPEED), MAX_SPEED);
        
        // Move robot using field-relative control
        drive.moveToPose(new Pose2d(-yMove, -xMove, 0));  // Forward/back, left/right, no rotation
        slide.setTurnAngleDeg(visionValues[2]);
    }
    
    @Override
    public void execute() {
        // Get vision alignment values
        
    }
    
    @Override
    public boolean isFinished() {
        // double[] visionValues = vision.getFineSearchVal();
        // return Math.abs(visionValues[0]) < TOLERANCE && 
        //        Math.abs(visionValues[1]) < TOLERANCE;
        return true;
    }
    
    @Override
    public void end(boolean interrupted) {
        //drive.moveRobot(0, 0, 0);  // Stop all movement
    }
}
