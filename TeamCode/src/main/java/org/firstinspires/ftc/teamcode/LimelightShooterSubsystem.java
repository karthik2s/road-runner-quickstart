package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/**
 * Reusable Subsystem for Limelight 3A Vision, AprilTag tracking, distance calculation,
 * dynamic flywheel RPM calculation for Pollen & Nectar ball types, and shifter/hood servo angle calculation.
 */
@Config
public class LimelightShooterSubsystem {

    // Configuration Constants
    public static int APRILTAG_PIPELINE_INDEX = 0;
    public static double LIMELIGHT_LENS_HEIGHT_INCHES = 12.0;
    public static double TARGET_CENTER_HEIGHT_INCHES = 36.0;
    public static double LIMELIGHT_MOUNT_ANGLE_DEG = 45.0;     // 45 degree upward camera mounting pitch angle
    public static int DESIRED_TAG_ID = -1;                 // -1 targets any AprilTag

    public static double TURRET_AUTO_AIM_GAIN = 0.30;      // Gain for turret tracking
    public static double AIM_DEADBAND_DEG = 0.75;           // Deadband tolerance to prevent hunting/oscillations
    public static double TURRET_MIN_ANGLE_DEG = -90.0;
    public static double TURRET_MAX_ANGLE_DEG = 90.0;

    // Ball Type Flywheel RPM Curves (Pollen = Smaller Ball, Nectar = Bigger Ball)
    public static double POLLEN_BASE_RPM = 2850.0;         // Base RPM for Pollen (Smaller ball)
    public static double POLLEN_RPM_SLOPE = 12.0;          // RPM increase per inch of distance
    public static double NECTAR_BASE_RPM = 2850.0;         // Base RPM for Nectar (Bigger ball)
    public static double NECTAR_RPM_SLOPE = 15.0;          // RPM increase per inch of distance

    public static double FLYWHEEL_MIN_RPM = 2000.0;
    public static double FLYWHEEL_MAX_RPM = 4500.0;

    public static double SHIFTER_OPEN_POSITION = 0.25;
    public static double SHIFTER_CLOSED_POSITION = 0.95;

    // Hardware
    private Limelight3A limelight;

    // State Variables
    private boolean targetVisible = false;
    private double tx = 0.0;
    private double ty = 0.0;
    private double calculatedDistanceInches = 0.0;
    private int detectedTagId = -1;

    public LimelightShooterSubsystem(HardwareMap hardwareMap) {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Exception ignored) {
            limelight = null;
        }

        if (limelight == null) {
            List<Limelight3A> limelights = hardwareMap.getAll(Limelight3A.class);
            if (limelights != null && !limelights.isEmpty()) {
                limelight = limelights.get(0);
            }
        }

        if (limelight != null) {
            limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
            limelight.start();
        }
    }

    /**
     * Polls the Limelight 3A for the latest frame results and updates vision metrics.
     * Should be called once per loop iteration.
     */
    public void update() {
        LLResult result = (limelight != null) ? limelight.getLatestResult() : null;

        targetVisible = false;
        tx = 0.0;
        ty = 0.0;
        detectedTagId = -1;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int fid = fiducial.getFiducialId();
                    if (DESIRED_TAG_ID < 0 || fid == DESIRED_TAG_ID) {
                        targetVisible = true;
                        detectedTagId = fid;
                        tx = fiducial.getTargetXDegrees();
                        ty = fiducial.getTargetYDegrees();
                        break;
                    }
                }
            }

            if (!targetVisible && result.getTa() > 0) {
                targetVisible = true;
                tx = result.getTx();
                ty = result.getTy();
            }

            if (targetVisible) {
                double angleToTargetRad = Math.toRadians(LIMELIGHT_MOUNT_ANGLE_DEG + ty);
                double heightDifference = TARGET_CENTER_HEIGHT_INCHES - LIMELIGHT_LENS_HEIGHT_INCHES;

                if (Math.tan(angleToTargetRad) > 0) {
                    double rawDistance = heightDifference / Math.tan(angleToTargetRad);
                    // Constrain calculated distance strictly within FTC field bounds (12" to 144" max)
                    calculatedDistanceInches = Range.clip(rawDistance, 12.0, 144.0);
                }
            }
        }
    }

    public boolean isLimelightConnected() {
        return limelight != null;
    }

    public boolean isTargetVisible() {
        return targetVisible;
    }

    public boolean isAimed() {
        return targetVisible && Math.abs(tx) <= AIM_DEADBAND_DEG;
    }

    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    public double getCalculatedDistanceInches() {
        return calculatedDistanceInches;
    }

    public int getDetectedTagId() {
        return detectedTagId;
    }

    /**
     * Calculates the new target angle for the turret to align with the AprilTag,
     * constrained to soft limits [-90°, +90°]. Includes deadband filtering.
     */
    public double getTurretTargetAngle(double currentTurretAngleDeg) {
        if (!targetVisible || Math.abs(tx) <= AIM_DEADBAND_DEG) {
            return currentTurretAngleDeg;
        }
        double desiredTurretAngle = currentTurretAngleDeg - (tx * TURRET_AUTO_AIM_GAIN);
        return Range.clip(desiredTurretAngle, TURRET_MIN_ANGLE_DEG, TURRET_MAX_ANGLE_DEG);
    }

    /**
     * Calculates required flywheel RPM dynamically based on AprilTag distance and ball type:
     * - isNectarMode = false (Pollen / Smaller Ball)
     * - isNectarMode = true (Nectar / Bigger Ball)
     */
    public double getFlywheelTargetRPM(boolean isNectarMode) {
        double baseRPM = isNectarMode ? NECTAR_BASE_RPM : POLLEN_BASE_RPM;
        if (!targetVisible || calculatedDistanceInches <= 0) {
            return baseRPM;
        }
        double slope = isNectarMode ? NECTAR_RPM_SLOPE : POLLEN_RPM_SLOPE;
        double calculatedRPM = baseRPM + (slope * calculatedDistanceInches);
        return Range.clip(calculatedRPM, FLYWHEEL_MIN_RPM, FLYWHEEL_MAX_RPM);
    }

    /**
     * Calculates required shifter/hood servo position dynamically based on distance to the AprilTag.
     */
    public double getShifterTargetPosition() {
        if (!targetVisible || calculatedDistanceInches <= 0) return SHIFTER_CLOSED_POSITION;
        double minDistance = 24.0;
        double maxDistance = 120.0;
        double normDistance = Range.clip((calculatedDistanceInches - minDistance) / (maxDistance - minDistance), 0.0, 1.0);
        return SHIFTER_CLOSED_POSITION - normDistance * (SHIFTER_CLOSED_POSITION - SHIFTER_OPEN_POSITION);
    }

    /**
     * Stops Limelight polling on OpMode shutdown.
     */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
