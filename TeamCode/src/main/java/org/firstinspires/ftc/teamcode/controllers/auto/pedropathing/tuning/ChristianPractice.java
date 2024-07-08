package org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.tuning;

import static org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.tuning.StraightBackAndForth.DISTANCE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.controllers.auto.pedropathing.pathGeneration.Point;

@Config
@Autonomous (name = "Christian Practice", group = "Autonomous Practice Coding")
public class ChristianPractice extends OpMode {
    private Telemetry telemetryC;
    public static double distance = 10;
    private boolean forward = true;
    private Follower follower;
    private Path forwards;
    private Path backward;

    @Override
    public void init(){
        follower = new Follower(hardwareMap);

        forwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(DISTANCE,0, Point.CARTESIAN)));
        forwards.setConstantHeadingInterpolation(0);
        backward = new Path(new BezierLine(new Point(DISTANCE,0, Point.CARTESIAN), new Point(DISTANCE,0,Point.CARTESIAN)));
        backward.setConstantHeadingInterpolation(0);

        follower.followPath(forwards);
        telemetryC = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryC.addLine("This will run the robot in a straight line going " + DISTANCE
                + " inches forward. The robot will go forward and backward continuously"
                + " along the path. Make sure you have enough room.");

        telemetryC.update();
    }
    @Override
    public void loop(){
        follower.update();
        if (!follower.isBusy()){
            if(forward){
                forward = false;
                follower.followPath(backward);
            }else{
                forward = true;
                follower.followPath(forwards);
            }
        }
        telemetryC.addData("going forward", forward);
        follower.telemetryDebug(telemetryC);
    }
}
