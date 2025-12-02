import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//Gives the OpMode the name, "ExampleCode" and displays it on the OpMode section of the Driver Hub
@TeleOp(name = "ExampleCode")
public class ExampleCode extends LinearOpMode {

//Declares a new DcMotor named, "Motor1"
    private DcMotor Motor1;

//When the OpMode is selected the following things will run
    @Override
    public void runOpMode(){

 //Defines Motor1 to be a DcMotor named, "Motor1" that is connected to the Robot Controller
        Motor1 = hardwareMap.get(DcMotor.class, "Motor1");

//Waits for the OpMode to be started
        waitForStart();

//Once the OpMode is started the following things will run
        if(opModeIsActive()){
            while(opModeIsActive()){

//If the dpad down button is held down Motor1 will Spin
                if(gamepad1.dpad_down){
                    Motor1.setPower(1);

//If the dpad down button is released Motor1 will Stop
                }else{
                    Motor1.setPower(0);
                }
            }
        }
    }
}
