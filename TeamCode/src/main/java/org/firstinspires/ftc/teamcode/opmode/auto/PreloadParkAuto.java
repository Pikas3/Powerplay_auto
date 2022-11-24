package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.SleeveDetector;
import org.firstinspires.ftc.teamcode.subsystem.lift.LiftConstants;

@Autonomous
public class PreloadParkAuto extends LinearOpMode {
    Pose2d START_POSE = new Pose2d(36, -64, Math.toRadians(90));
    Pose2d PARKINGSTART_POSE = new Pose2d(36, -52, Math.toRadians(90));
    Robot robot;
    SleeveDetector detector = new SleeveDetector();
    SleeveDetection.Color parkingPos = SleeveDetection.Color.MAGENTA;
    private double turret_front = 0;
    private double turret_back = 240;
    private double turret_right = 120;
    private double turret_left = -120;

    public TrajectorySequence build_preload_sequence(double liftHeight) {
        Vector2d JUNCTION_POSE = new Vector2d(40, -52);
        TrajectorySequence preload = robot.drive.trajectorySequenceBuilder(START_POSE)
            .splineTo(JUNCTION_POSE, Math.toRadians(30))
            .addTemporalMarker(3.3, () -> {
                // lift
                //robot.lift.setTargetHeight(1);
            })
            .addTemporalMarker(4.8, () -> {
                //robot.lift.setArmPos(LiftConstants.IdleArm);
            })
            .addTemporalMarker(6.2, () -> {
                //robot.lift.setClaw1Pos(LiftConstants.CLAWOPENPOS1);
            })

            //.setReversed(false)
            .lineToLinearHeading(PARKINGSTART_POSE)
            .addTemporalMarker(13.5, () -> {
                  // Back to moving  stage setting
//                robot.lift.setTargetHeight(0);
//                robot.turret.setPosition(RobotConstants.FrontTurret);

            })
            .build();

        return preload;
    }

    public void robot_init() {
        robot.lift.setTargetHeight(1);
        robot.lift.setTargetRotation(turret_front);
        robot.lift.setArmPos(LiftConstants.IdleArm);
        robot.lift.setClaw1Pos(LiftConstants.CLAWOPENPOS1);
    }
    public void robot_prepare_move() {
        robot.lift.setTargetHeight(1);
        robot.lift.setTargetRotation(turret_front);
        robot.lift.setArmPos(LiftConstants.IdleArm);
        robot.lift.setClaw1Pos(LiftConstants.CLAWCLOSEPOS1);
    }
//    public void robot_drop_cone(double height) {
//        robot.lift.setTargetHeight(height);
//        robot.lift.setTargetRotation(turret_front);  //?
//        robot.lift.setArmPos(LiftConstants.IntakingArm);
//        robot.lift.setClaw1Pos(LiftConstants.CLAWOPENPOS1);
//    }

    public void runOpMode() {
        robot = new Robot(telemetry, hardwareMap);
        ElapsedTime timer = new ElapsedTime();
        detector.init(hardwareMap, telemetry);

        // lift height for junction = 0.5?
        TrajectorySequence preloadSeq = build_preload_sequence(0.5);

        TrajectorySequence parking1 = robot.drive.trajectorySequenceBuilder(PARKINGSTART_POSE)
                .lineToLinearHeading(new Pose2d(36, -34, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(12, -34, Math.toRadians(90)))
                .build();
        TrajectorySequence parking2 = robot.drive.trajectorySequenceBuilder(PARKINGSTART_POSE)
                .lineToLinearHeading(new Pose2d(36, -34, Math.toRadians(90)))
                .build();
        TrajectorySequence parking3 = robot.drive.trajectorySequenceBuilder(PARKINGSTART_POSE)
                .lineToLinearHeading(new Pose2d(36, -34, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(60, -34, Math.toRadians(90)))
                .build();

        robot.drive.setPoseEstimate(START_POSE);

        robot.init();
        // Waiting for start
        int counter = 0;
        while (!isStarted() && !isStopRequested()) {
            parkingPos = detector.getColor();
            telemetry.addData("Parking position", parkingPos);
            telemetry.addData("counter", counter);
            counter++;
            telemetry.update();
        }

        // Start...
        detector.stop();
        //waitForStart();

        // Preload
        robot.drive.followTrajectorySequenceAsync(preloadSeq);
        // wait for preload sequence
        timer.reset();
        while(!isStopRequested() && robot.drive.isBusy() && opModeIsActive()) {
            robot.update();
            if(timer.milliseconds() > 12800) {
                robot.drive.stopTrajectory();
            }
        }

        // Parking
        if (parkingPos == SleeveDetection.Color.MAGENTA) {
            robot.drive.followTrajectorySequenceAsync(parking1);
        } else if ( parkingPos == SleeveDetection.Color.BLUE) {
            robot.drive.followTrajectorySequenceAsync(parking2);
        } else if (parkingPos == SleeveDetection.Color.RED) {
            robot.drive.followTrajectorySequenceAsync(parking3);
        }

        // Wait for stop
        while (!isStopRequested() && robot.drive.isBusy() && opModeIsActive()) {
            telemetry.update();
            robot.update();
        }

    }

}
