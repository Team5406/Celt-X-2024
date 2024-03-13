// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot.subsystems.vision;

import java.util.List;
import java.util.ListIterator;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Predicate;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

/** Create a camera */
public class AprilTagCamera implements Runnable, AutoCloseable {
  private final double APRILTAG_POSE_AMBIGUITY_THRESHOLD = 0.2;

  public enum Resolution {
    RES_320_240(320, 240),
    RES_640_480(640, 480),
    RES_1280_720(1280, 720),
    RES_1280_800(1280, 800),
    RES_1920_1080(1920, 1080);

    public final int width;
    public final int height;

    private Resolution(int width, int height) {
      this.width = width;
      this.height = height;
    }
  }

  private PhotonCamera m_camera;
  private PhotonCameraSim m_cameraSim;
  private PhotonPoseEstimator m_poseEstimator;
  private Transform3d m_transform;
  private AtomicReference<EstimatedRobotPose> m_atomicEstimatedRobotPose;
  private AtomicReference<PhotonPipelineResult> m_atomicPipeline;

  /**
   * Create VisionCamera
   * @param name Name of device
   * @param transform Location on robot in meters
   * @param resolution Resolution used by camera
   * @param fovDiag Diagonal FOV of camera
   */
  public AprilTagCamera(String name, Transform3d transform, Resolution resolution, Rotation2d fovDiag) {
    this.m_camera = new PhotonCamera(name);
    this.m_transform = transform;
    var fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // PV estimates will always be blue
    fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    //this.m_poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera, m_transform);
    //m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    this.m_atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();
    this.m_atomicPipeline = new AtomicReference<PhotonPipelineResult>();
    
    // Create simulated AprilTag camera
    var cameraProperties = SimCameraProperties.PERFECT_90DEG();
    cameraProperties.setCalibration(resolution.width, resolution.height, fovDiag);
    this.m_cameraSim = new PhotonCameraSim(m_camera, cameraProperties);

    // Enable wireframe in sim camera stream
    m_cameraSim.enableDrawWireframe(true);
  }

  /**
   * Get camera sim
   * @return Simulated camera object
   */
  PhotonCameraSim getCameraSim() {
    return m_cameraSim;
  }

  @Override
  public void run() {
    // Return if camera or field layout failed to load
    if (/*m_poseEstimator == null ||*/m_camera == null) return;

    // Update and log inputs
    PhotonPipelineResult pipelineResult = m_camera.getLatestResult();
    // Return if result is non-existent or invalid
    if (!pipelineResult.hasTargets()) return;

    m_atomicPipeline.set(pipelineResult);
  }

  public PhotonPipelineResult getLatestPipelineResult() {
    return m_atomicPipeline.getAndSet(null);
  }

  /**
   * Allows user to select the active pipeline index
   * @param index The active pipeline index
   */
  public void setPipelineIndex(int index) {
    m_camera.setPipelineIndex(index);
  }

  /**
   * Get camera to robot transform
   * @return Camera to robot transform
   */
  public Transform3d getTransform() {
    return m_transform;
  }

  @Override
  public void close() {
    m_camera.close();
  }
}