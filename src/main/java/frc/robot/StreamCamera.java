package frc.robot;

// imports for camera stream
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;

// Code for getting the USB camera(logiTech) onto the driver station
public class StreamCamera extends TimedRobot {

    public void robotInit() {
        CameraServer.startAutomaticCapture();
        System.out.println("testing this above code work");
        
    }

  }
