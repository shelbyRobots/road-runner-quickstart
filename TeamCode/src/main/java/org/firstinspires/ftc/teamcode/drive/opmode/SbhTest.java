package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@Autonomous(group = "drive")
public class SbhTest extends LinearOpMode {

    public static enum DROP_POS
    {
        DROPA,
        DROPB,
        DROPC
    }

    public static DROP_POS dpos = DROP_POS.DROPB;

    Pose2d wA1Pose = new Pose2d(4, -59,0);
    Pose2d wA2Pose = new Pose2d(10, -44,-Math.toRadians(90));
    Pose2d wB1Pose = new Pose2d(24, -48, Math.toRadians(45));
    Pose2d wB2Pose = new Pose2d(24, -30,-Math.toRadians(20));
    Pose2d wC1Pose = new Pose2d(48, -59,0);
    Pose2d wC2Pose = new Pose2d(54, -46,-Math.toRadians(45));

    Pose2d srtPose = new Pose2d(-61.5,-44,0);
    Pose2d dogPose = new Pose2d(0,-59,0);
    Pose2d wAcPose = new Pose2d(-4.0,-59,0);
    Pose2d shtPose = new Pose2d(-2.0,-36,0);
    Pose2d md1Pose = new Pose2d(-10.0, -52.0, 0);
    Pose2d walPose = new Pose2d(-60.0,-52,0);
    Pose2d w2gPose = new Pose2d(-61.0,-24,0);
    Pose2d prkPose = new Pose2d(10.0,-36,0);
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d w1dPose;
        Pose2d w2dPose;

        switch (dpos)
        {
            case DROPB:
                w1dPose = wB1Pose;
                w2dPose = wB2Pose;
                break;
            case DROPC:
                w1dPose = wC1Pose;
                w2dPose = wC2Pose;
                break;
            case DROPA:
            default:
                w1dPose = wA1Pose;
                w2dPose = wA2Pose;
        }

        drive.setPoseEstimate(srtPose);

        Trajectory t1 = drive.trajectoryBuilder(srtPose)
            .splineTo(dogPose.vec(), dogPose.getHeading())
            .splineTo(w1dPose.vec(), w1dPose.getHeading()).build();
        Trajectory t2 = drive.trajectoryBuilder(t1.end())
            .splineToLinearHeading(wAcPose,0).build();
        Trajectory t3 = drive.trajectoryBuilder(t2.end())
            .strafeTo(shtPose.vec()).build();
        Trajectory t4 = drive.trajectoryBuilder(t3.end())
            .strafeTo(md1Pose.vec()).build();
        Trajectory t5 = drive.trajectoryBuilder(t4.end())
            .strafeTo(walPose.vec()).build();
        Trajectory t6 = drive.trajectoryBuilder(t5.end())
            .strafeTo(w2gPose.vec()).build();
        Trajectory t7 = drive.trajectoryBuilder(t6.end())
            .splineTo(w2dPose.vec(), w2dPose.getHeading()).build();
        Trajectory t8 = drive.trajectoryBuilder(t7.end(),true)
            .splineToLinearHeading(prkPose, 0).build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(t1);
        drive.followTrajectory(t2);
        drive.followTrajectory(t3); sleep(2000);
        drive.followTrajectory(t4);
        drive.followTrajectory(t5);
        drive.followTrajectory(t6);
        drive.followTrajectory(t7);
        drive.followTrajectory(t8);

        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
