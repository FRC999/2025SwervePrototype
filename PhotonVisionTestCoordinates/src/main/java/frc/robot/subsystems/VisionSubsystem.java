// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems; 

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    /** Creates a new VisionSubsystem. */

    
    private static PhotonCamera camera;
    public VisionSubsystem() {
    }

    public void GetCoordinatesFromCamera() {
            // Initialize the PhotonCamera with the name of the camera (as configured in
            // PhotonVision)
            camera = new PhotonCamera("Arducam_OV9782_USB_Camera (1)");

            // Main loop to continuously get the latest AprilTag coordinates
            while (true) {
                // Get the latest result from the camera
                PhotonPipelineResult result = camera.getLatestResult();

                // Check if the result has any targets (AprilTags)
                if (result.hasTargets()) {
                    // Get the best target (the one with the lowest ambiguity)
                    PhotonTrackedTarget bestTarget = result.getBestTarget();

                    // Get the AprilTag ID
                    int aprilTagId = bestTarget.getFiducialId();
                    System.out.println("Detected AprilTag ID: " + aprilTagId);

                    // Get the 3D pose of the AprilTag relative to the camera
                    Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();
                    double x = cameraToTarget.getX(); // X coordinate (forward/backward)
                    double y = cameraToTarget.getY(); // Y coordinate (left/right)
                    double z = cameraToTarget.getZ(); // Z coordinate (up/down)

                    // Print the coordinates to the console
                    System.out.println("AprilTag Coordinates (X, Y, Z): (" + x + ", " + y + ", " + z + ")");

                    // You can also get the rotation components (roll, pitch, yaw) if needed
                    double yaw = cameraToTarget.getRotation().getZ();
                    double pitch = cameraToTarget.getRotation().getY();
                    double roll = cameraToTarget.getRotation().getX();
                    System.out.println("Rotation (Yaw, Pitch, Roll): (" + yaw + ", " + pitch + ", " + roll + ")");
                } else {
                    System.out.println("No AprilTags detected!");
                }

                // Sleep for a short period to avoid overwhelming the console
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
