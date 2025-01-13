package frc.robot.subsystems.VisionSubsystem;

public void updateInputs(VisionIO.VisionIOInputs inputs) {
    inputs.estimatedPose =
            VisionSubsystem.etimatedRobotPose;
    inputs.connected =
            VisionSubsystem.camera.isConnected();
    inputs.PhotonTrackedTarget =
            VisionSubsystem.target
    inputs.results =