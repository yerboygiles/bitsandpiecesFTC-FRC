
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="Basic: Autonomous Theo", group="Autonomous")

public class HallFollower extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    BNO055IMU imu;
    
    Orientation angles;
    Acceleration gravity;
    
    ModernRoboticsI2cRangeSensor rangeR;

    ModernRoboticsI2cRangeSensor rangeL;

    ModernRoboticsI2cRangeSensor rangeF;

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    
    double drivePower;
    double leftPower;
    double rightPower;
    
    int currentPos;
    double kPu = 1;
    double kIu = 1;
    double kDu = 1;
    
    double Pu=0;
    double Iu=0;
    double Du=0;
    
    double PIDu=0;
    
    double kPz = .03;
    double kIz = .04;
    double kDz = 1;
    
    double Pz=0;
    double Iz=0;
    double Dz=0;
    
    double PIDz=0;
    
    double kPd = 1;
    double kId = 1;
    double kDd = 1;
    
    double Pd=0;
    double Id=0;
    double Dd=0;
    
    double PIDd=0;
    
    double zangle;
    
    double Tz=0;
    
    double zangleError;
    
    double zangleErrorPrev;
    
    double zangleCumulative;
    
    double driveError;
    
    double Tu=0;
    
    double ultraError;
    
    double ultraErrorPrev;
    
    double ultraCumulative;
    
    int state;
    int stateChild;
    
    double frontDist;
    double rightDist;
    double leftDist;
    
    @Override
    public void runOpMode() {
        state=1;
        stateChild=0;
        initialize();
        waitForStart();
        ultraSensors();
        resetMot();
        while (opModeIsActive()) {
            currentPos=(Math.abs(leftDrive.getCurrentPosition())+Math.abs(rightDrive.getCurrentPosition()))/2;
            //stateMach();
            if(state==1){
                PIDzangle(0);
                PIDencoder(4200,currentPos);
                encDrive(4200, PIDz, PIDd, PIDu);
                if(currentPos>=4200){
                    state++;
                    resetMot();
                }
            }
            if(state==2){
                PIDzangle(-90);
                PIDencoder(400,currentPos);
                encDrive(400,PIDz,PIDd,PIDu);
                if(currentPos>=400){
                    state++;
                    resetMot();
                }
            }
            if(state==2){
                PIDzangle(-90);
                PIDencoder(5000,currentPos);
                encDrive(5000,PIDz,PIDd,PIDu);
                if(currentPos>=5000){
                    state++;
                    resetMot();
                }
            }
            angles    = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            zangle    =angles.firstAngle;
            telemetry.addData("state",state);
            telemetry.addData("zangle",zangle);
            telemetry.update();
        }
    }
    private void ultraSensors(){
        frontDist = rangeF.getDistance(DistanceUnit.CM);
        leftDist  = rangeR.getDistance(DistanceUnit.CM);
        rightDist = rangeL.getDistance(DistanceUnit.CM);
    }
    private void PIDencoder(double wantedPosPID,double currentPosPID){
        driveError=wantedPosPID-(currentPosPID);
        Pd=(kPd*driveError)/wantedPosPID;
        Id=0;
        Dd=0;
        PIDd=Pd+Id+Dd;
    }
    private void PIDzangle(double wantedAngle){
        zangleError=wantedAngle-zangle;
        zangleCumulative+=zangleError;
        Tz+=1;
        // if(zangleError<0&&zangleCumulative>0){
        //     zangleCumulative=0;
        //     Tz=0;
        // }
        // if(zangleError>0&&zangleCumulative<0){
        //     zangleCumulative=0;
        //     Tz=0;
        // }
        Pz=(kPz*zangleError);
        //if((Tz!=0)&&(zangleCumulative!=0)){}
        Iz=kIz*(zangleCumulative/Tz);
        Dz=0;
        PIDz=(Pz+Iz+Dz);
        telemetry.addData("average z",Iz);
    }
    private void PIDultraH(){
        ultraSensors();
        ultraError=leftDist-100;
        ultraCumulative+=ultraError;
        Tu+=1;
        Pu=(kPu*ultraError)/100;
        Iu=0;
        // if(ultraError>10||ultraError<-10){
        //     Iu=(kIz*(ultraCumulative/Tu));
        // }
        PIDu=Pu+Iu;
    }
    private boolean encDrive(double wantedPos,double zanglePID,double encoderPID,double ultraPID){
        double slowPoint=(wantedPos/5)*4;
        currentPos=(Math.abs(leftDrive.getCurrentPosition())+Math.abs(rightDrive.getCurrentPosition()))/2;
        if(currentPos>wantedPos){
            leftPower=0;
            rightPower=0;
        }
        else if(currentPos>=slowPoint){
            PIDencoder(wantedPos-slowPoint,currentPos-slowPoint);
            leftPower=((encoderPID/4)-(zanglePID)/1)+.1;
            rightPower=((encoderPID/4)+(zanglePID)/1)+.1;
        }
        else if(currentPos<wantedPos){
            leftPower=.2-(zanglePID)/1;
            rightPower=.2+(zanglePID)/1;
        }
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        telemetry.addData("Right Enc",rightDrive.getCurrentPosition());
        telemetry.addData("Left Enc",leftDrive.getCurrentPosition());
        telemetry.addData("right",rightPower);
        telemetry.addData("left",leftPower);
        telemetry.addData("pidz",zanglePID);
        
        return (leftPower==0||rightPower==0);
    }
    private void imuinit(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }
    private void initialize() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        rangeF = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RangeF");
        rangeR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RangeR");
        rangeL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RangeL");

        leftDrive  = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imuinit();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    private void resetMot(){
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
