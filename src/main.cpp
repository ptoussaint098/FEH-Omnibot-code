#include <FEH.h>
#include <Arduino.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "FEHServo.h"
#include "FEHSD.h"


#define wheel_radius 1.25    //inches
#define robot_radius 3.625   // inches
#define wheel1_theta 0     // degrees
#define wheel2_theta 120.0     // degrees
#define wheel3_theta 240.0     // degrees
#define Pi 3.1415926535897
#define robot_weight 1.079        //kilograms
#define countsperinch 40.490175226    //counts per inch
#define countsperrotation 318.0 //counts per rotation
#define motormaxrpm 150
#define momentumfactor 10

#define motor_torque_weight 2.0


//PID Constants opne to be tweaked
#define Pid_P_Constant  1
#define Pid_D_Constant  0
#define Pid_I_Constant  0
//servo mins and maxes
#define SERVO_MIN 500
#define SERVO_MAX 1424



// Motor Defines
FEHMotor motor1(FEHMotor::Motor1, 9.0);
FEHMotor motor2(FEHMotor::Motor3, 9.0);
FEHMotor motor3(FEHMotor::Motor0, 9.0);
//Encoder Defines
DigitalEncoder encoder1(FEHIO::Pin13);
DigitalEncoder encoder2(FEHIO::Pin14);
DigitalEncoder encoder3(FEHIO::Pin10);
//CDS cell
AnalogInputPin cds_cell(FEHIO::Pin8);
//Servo
FEHServo arm(FEHServo::Servo0);
FEHServo compost(FEHServo::Servo7);





class robot{
    public:
    //accepts arguements for degrees at any angle relative to robot
    //dist is in inches
    //speed is in inches per second
    void move(float dist, float angle, float speed)
    {
       //RESET 
        encoder1.ResetCounts();
        encoder2.ResetCounts();
        encoder3.ResetCounts();
        //for move function leftover code
        loop_speed=speed;

        //calculates a relative velocity
        velocity_x=cos((angle/180.0)*Pi)*speed;
        velocity_y=sin((angle/180.0)*Pi)*speed;

       //calculates correct wheel velocities
        wheelspeedcalc(velocity_x,velocity_y, 0);
        //calculates encoder counts from time and wheelspeed
        encoder1dstcount=((((fabs(wheelspeedrpm1)*wheel_radius*(2.0*Pi))/60)*(dist/speed))*countsperinch);
        encoder2dstcount=((((fabs(wheelspeedrpm2)*wheel_radius*(2.0*Pi))/60)*(dist/speed))*countsperinch);
        encoder3dstcount=((((fabs(wheelspeedrpm3)*wheel_radius*(2.0*Pi))/60)*(dist/speed))*countsperinch);
        //ensures that if the encount is low then it will not be as low since this exit the loop early from previous movement
        if (encoder1dstcount<15)
        {
            encoder1dstcount+=20;
        }
        if (encoder2dstcount<15)
        {
            encoder2dstcount+=20;
        }
        if (encoder3dstcount<15)
        {
            encoder3dstcount+=20;
        }

       
        //resets the PID variables from previos calls of the move function
        pidreset();
       
        //loop unti la wheel reaches the correct destination
        while ((encoder1.Counts()<=encoder1dstcount)&&(encoder2.Counts()<=encoder2dstcount)&&(encoder3.Counts()<=encoder3dstcount))
        {
            //sets the motors to the adequete voltage
            motor1.SetPercent(motor1_voltage);
            motor2.SetPercent(motor2_voltage);
            motor3.SetPercent(motor3_voltage);
   
        }

   
    }
    //calculates corect wheelspeed for each wheel, should not be called in main
    void wheelspeedcalc(float vx, float vy, float botrot)
    {
       //calculates the rpm of each wheel
        wheelspeedrpm1 = ((sin((wheel1_theta/360.0)*2.0*Pi)*vx+-1.0*cos((wheel1_theta/360.0)*2.0*Pi)*vy+(-1.0*robot_radius*botrot))/wheel_radius)*(60.0/(2.0*Pi));
        wheelspeedrpm2 = ((sin((wheel2_theta/360.0)*2*Pi)*vx+-1.0*cos((wheel2_theta/360.0)*2.0*Pi)*vy+(-1.0*robot_radius*botrot))/wheel_radius)*(60.0/(2.0*Pi));
        wheelspeedrpm3 = ((sin((wheel3_theta/360.0)*2.0*Pi)*vx+-1.0*cos((wheel3_theta/360.0)*2.0*Pi)*vy+(-1.0*robot_radius*botrot))/wheel_radius)*(60.0/(2.0*Pi));

        //calculates correct voltage for each wheel based on the current max rpm under load
        motor1_voltage = ((wheelspeedrpm1/motormaxrpm)*100);
        motor2_voltage = ((wheelspeedrpm2/motormaxrpm)*100);
        motor3_voltage = ((wheelspeedrpm3/motormaxrpm)*100);
        FEHLog::printf("Desired Voltage1: %f, Desired Voltage2: %f, Desired Voltage3: %f \n",(double)motor1_voltage,(double)motor2_voltage,(double)motor3_voltage);


    }
    //calculates the PID should not be called in main
    void pidcalc()
    {
        //finds the time between calls
        time_next_pid=millis();
        time_diff_pid=time_next_pid-last_time_pid;

        //sets the current encoder counts
        encoder1now=encoder1.Counts();
        encoder2now=encoder2.Counts();
        encoder3now=encoder3.Counts();


        //finds the wheel speed based on the encoders
        actual_wheel_speed1=((encoder1now-encoder1last)/countsperinch)/(time_diff_pid*.001);
        actual_wheel_speed2=((encoder2now-encoder2last)/countsperinch)/(time_diff_pid*.001);
        actual_wheel_speed3=((encoder3now-encoder3last)/countsperinch)/(time_diff_pid*.001);

        //finds the actual individual wheel RPM which is mostly for troubleshooting
        actualwheelspeedrpm1=(actual_wheel_speed1*60.0)/(2*Pi);
        actualwheelspeedrpm2=(actual_wheel_speed2*60.0)/(2*Pi);
        actualwheelspeedrpm3=(actual_wheel_speed3*60.0)/(2*Pi);

        //finds error between actual and projected
        pid_error1=((fabs(wheelspeedrpm1)*Pi*wheel_radius*2.0)/60.0)-actual_wheel_speed1;
        pid_error2=((fabs(wheelspeedrpm2)*Pi*wheel_radius*2.0)/60.0)-actual_wheel_speed2;
        pid_error3=((fabs(wheelspeedrpm3)*Pi*wheel_radius*2.0)/60.0)-actual_wheel_speed3;

        //finds error in rpm which is for tourbleshooting
        pid_errorrpm1= (pid_error1*60)/(Pi*wheel_radius*2.0);
        pid_errorrpm2= (pid_error2*60)/(Pi*wheel_radius*2.0);
        pid_errorrpm3= (pid_error3*60)/(Pi*wheel_radius*2.0);

        //finds the error in the voltage which is alspo for trouble shooting
        pid_volterr1=((pid_errorrpm1/motormaxrpm)*100.0);
        pid_volterr2=((pid_errorrpm2/motormaxrpm)*100.0);
        pid_volterr3=((pid_errorrpm3/motormaxrpm)*100.0);

        FEHLog::printf("error %f", (double)pid_volterr3);

        //finds the sum of the errors
        pid_sumoferrors1+=pid_volterr1*(time_diff_pid*.001);
        pid_sumoferrors2+=pid_volterr2*(time_diff_pid*.001);
        pid_sumoferrors3+=pid_volterr3*(time_diff_pid*.001);

        //calculates corresponding P, I, and D terms
        pid_Pterm1= Pid_P_Constant*pid_volterr1;
        pid_Pterm2= Pid_P_Constant*pid_volterr2;
        pid_Pterm3= Pid_P_Constant*pid_volterr3;

        pid_Iterm1=Pid_I_Constant*pid_sumoferrors1;
        pid_Iterm2=Pid_I_Constant*pid_sumoferrors2;
        pid_Iterm3=Pid_I_Constant*pid_sumoferrors3;

        pid_Dterm1=Pid_D_Constant*((pid_volterr1-pid_lasterror1)/(time_diff_pid*.001));
        pid_Dterm2=Pid_D_Constant*((pid_volterr2-pid_lasterror2)/(time_diff_pid*.001));
        pid_Dterm3=Pid_D_Constant*((pid_volterr3-pid_lasterror3)/(time_diff_pid*.001));
       
       //Uses Terms for corrections
        motor1_voltage=pid_Pterm1+pid_Iterm1+pid_Dterm1+base_voltage1;
        motor2_voltage=pid_Pterm2+pid_Iterm2+pid_Dterm2+base_voltage2;
        motor3_voltage=pid_Pterm3+pid_Iterm3+pid_Dterm3+base_voltage3;
       

        //setting current values as previous values for next run
        last_time_pid=time_next_pid;

        encoder1last=encoder1now;
        encoder2last=encoder2now;
        encoder3last=encoder3now;

        pid_lasterror1=pid_volterr1;
        pid_lasterror2=pid_volterr2;
        pid_lasterror3=pid_volterr3;

    }
    //Simply resets PID to be used for new calls should not be called within main and should be called before move loops
    void pidreset()
    {
        time_next_pid=0;
       

        encoder1.ResetCounts();
        encoder2.ResetCounts();
        encoder3.ResetCounts();


        encoder1last=encoder1.Counts();
        encoder2last=encoder2.Counts();
        encoder3last=encoder3.Counts();

        pid_sumoferrors1=0;
        pid_sumoferrors2=0;
        pid_sumoferrors3=0;

        pid_lasterror1=0;
        pid_lasterror2=0;
        pid_lasterror3=0;

        base_voltage1=motor1_voltage;
        base_voltage2=motor2_voltage;
        base_voltage3=motor3_voltage;

   
        last_time_pid=millis();
    }
    //Function to turn by any degree in radians for a set period of time
    //For the degree negative is clockwise, positive is counter clockwise for the robot.
    void turn(float degree, float time)
    {
        encoder1.ResetCounts();
        encoder2.ResetCounts();
        encoder3.ResetCounts();
        //calculates the radians per second for rotation
        radpersec=((Pi*degree)/180)/time;
        //calls wheelspeed calc for said rotation
        wheelspeedcalc(0,0,radpersec);
        //calculates correct encoder amount
        encoder1dstcount=((((fabs(wheelspeedrpm1)*wheel_radius*(2.0*Pi))/60)*(time))*countsperinch)-momentumfactor*1.0;
        encoder2dstcount=((((fabs(wheelspeedrpm2)*wheel_radius*(2.0*Pi))/60)*(time))*countsperinch)-momentumfactor*1.0;
        encoder3dstcount=((((fabs(wheelspeedrpm3)*wheel_radius*(2.0*Pi))/60)*(time))*countsperinch)-momentumfactor*1.0;

        //ensures that momentum doesnt trigger encoder counts
        if (encoder1dstcount==15)
        {
            encoder1dstcount+=15;
        }
        if (encoder2dstcount==15)
        {
            encoder2dstcount+=15;
        }
        if (encoder3dstcount==15)
        {
            encoder3dstcount+=15;
        }


        //main loop until one reaches desired rotation
        while ((encoder1.Counts()<=encoder1dstcount)&&(encoder2.Counts()<=encoder2dstcount)&&(encoder3.Counts()<=encoder3dstcount))
        {
           
            motor1.SetPercent(motor1_voltage);
            motor2.SetPercent(motor2_voltage);
            motor3.SetPercent(motor3_voltage);

            

        }
       

    }
    //purely experimental function does not work due to RCS Error never call
    void gotopos(float x_pos, float y_pos, float heading)
    {
        RCSPose* class_position= RCS.RequestPosition();
        if (class_position== nullptr)
        {
            return;
        }

        diff_in_x=x_pos-class_position->x;
        diff_in_y=y_pos-class_position->y;
        diff_in_rot=heading-class_position->heading;
        


        dstmove=sqrtf(pow(diff_in_x,2.0)+(pow(diff_in_y,2.0)));
        if((atan2f(diff_in_y,diff_in_x))>=0)
        {
            rotmove=((atan2f(diff_in_y,diff_in_x))*(180/Pi))-6-class_position->heading;
        }
        else
        {
           rotmove=((atan2f(diff_in_y,diff_in_x))*(180/Pi))+360-6-class_position->heading;
        }
        if(rotmove>180)
        {
            rotmove-=360;
        }
        if(rotmove<-180)
        {
            rotmove+=360;
        }
       

        FEHLog::printf("Projected Movement: %f\n",dstmove);
        FEHLog::printf("Projected Rot: %f\n",rotmove);
       
        move(dstmove,rotmove,5);
       

    }
    //Moves the arm to a specific angle in a period of time
    //0 degrees is up no degree greater than 120 should be called
    void armmove(float angle, float time_to_complete)
    {
        //finds how much arm should move
        waitime=time_to_complete/(abs(angle-arm_angle));
        //Fixes angle
        if (arm_angle<angle)
        {
            //Moves arm until it reaches correct position
            while (arm_angle<angle)
            {
                arm_angle+=1.0;
                arm.SetDegree(arm_angle);
                
               
                Sleep(waitime);
            }
           
        }
        //Corrects for opposite movement
        else
        {
             while (arm_angle>angle)
            {
                arm_angle-=1.0;
                arm.SetDegree(arm_angle);
                Sleep(waitime);
                
            }

        }
       
    }
    //Purely used to allow for parallel code execution stops motors
    void stopmot()
    {
       
        motor3.SetPercent(0.0);
        motor2.SetPercent(0.0);
        motor1.SetPercent(0.0);
       
    }
    //Purely a function for testing values
    void writefuncs()
    {
       
       
        FEHLog::printf("Volt Error1: %f, Volt Error2: %f, Volt Error3: %f",(double)pid_volterr1,(double)pid_volterr2, (double)pid_volterr3);
        FEHLog::printf("Motor 1 volt: %f, Motor 2 volt: %f, Motor 3 volt: %f,",(double)motor1_voltage,(double)motor2_voltage,(double)motor3_voltage);

    }
    void datatrack()
    {
        static FEHFile *motordatapntr = SD.FOpen("Motordata.txt", "w");
        SD.FPrintf(motordatapntr,"%f\n%f\n%f\n",motor1_voltage,motor2_voltage,motor3_voltage);

    }
    //Purely experimental does not work due to RCS error
    void drivetill(float y_posapple)
    {   encoder1.ResetCounts();
        encoder2.ResetCounts();
        encoder3.ResetCounts();
        RCSPose *rcs_position_loop=RCS.RequestPosition();

        while ((rcs_position_loop->y < y_posapple))
        {
            encoder1dstcount=((((fabs(wheelspeedrpm1)*wheel_radius*(2.0*Pi))/60)*(.2/loop_speed))*countsperinch);
            encoder2dstcount=((((fabs(wheelspeedrpm2)*wheel_radius*(2.0*Pi))/60)*(.2/loop_speed))*countsperinch);
            encoder3dstcount=((((fabs(wheelspeedrpm3)*wheel_radius*(2.0*Pi))/60)*(.2/loop_speed))*countsperinch);
            while ((encoder1.Counts()<=encoder1dstcount)&&(encoder2.Counts()<=encoder2dstcount)&&(encoder3.Counts()<=encoder3dstcount))
            {


            }
            encoder1.ResetCounts();
            encoder2.ResetCounts();
            encoder3.ResetCounts();
            rcs_position_loop=RCS.RequestPosition();
            if (RCS.RequestsRemaining()<=25)
            {
                return;
            }

        }

        

        

    }
    //list of Vars
    private:
    float wheelspeed1, wheelspeed2, wheelspeed3;
    float motor1_voltage, motor2_voltage, motor3_voltage;   //in percent
    float wheelspeedrpm1, wheelspeedrpm2, wheelspeedrpm3;    // in rads/sec
    float encoder1dstcount, encoder2dstcount, encoder3dstcount;
    float velocity_x, velocity_y, rotation_rad;   // in inches per second and rotation of robot is in
    unsigned long int time_next_pid, last_time_pid, time_diff_pid;
    int encoder1last, encoder2last, encoder3last;
    int encoder1now, encoder2now, encoder3now;
    float pid_velocity1, pid_velocity2, pid_velocity3;
    float actual_wheel_speed1, actual_wheel_speed2, actual_wheel_speed3;    //linear velocities of each wheel
    float pid_error1, pid_error2, pid_error3;
    float pid_lasterror1, pid_lasterror2, pid_lasterror3;
    float pid_Pterm1, pid_Iterm1, pid_Dterm1, pid_Pterm2, pid_Iterm2, pid_Dterm2, pid_Pterm3, pid_Iterm3, pid_Dterm3;
    float pid_sumoferrors1, pid_sumoferrors2, pid_sumoferrors3;
    float base_voltage1, base_voltage2, base_voltage3;
    float radpersec;
    int index;
    float arm_angle=0;
    float waitime;
    float testtime;
    float actualwheelspeedrpm1,actualwheelspeedrpm2,actualwheelspeedrpm3;

    float diff_in_x, diff_in_y, diff_in_rot;
    float dstmove,rotmove;

    float pid_errorrpm1,pid_errorrpm2,pid_errorrpm3;
    float pid_volterr1, pid_volterr2, pid_volterr3;
    float loop_speed;


};







void ERCMain()
{
    //Vars used in Main
    int check=1;
    int x_touch, y_touch;
    float applebucket_y;
    char region_char;
    int region;

    //Enables output
    FEHLog::enableBLE(130);
    //initializes RCS
    RCS.InitializeTouchMenu("0910B7XJM");
    
    //Set Servo Min and Max
    arm.SetMax(SERVO_MAX);
    arm.SetMin(SERVO_MIN);
    
    robot robot;
   
    //Tests SD Card which rarely works
    FEHFile *filepntr = SD.FOpen("Test.txt","w");
    SD.FPrintf(filepntr,"Test");
    FEHLog::printf("Test");
    SD.FCloseAll();
    //FInds which region we are on
    region_char=RCS.CurrentRegionLetter();
    switch (region_char) {
        case 'A':
            region = 1;
            break;
        case 'B':
            region = 2;
            break;
        case 'C':
            region = 3;
            break;
        case 'D':
            region = 4;
            break;
        case 'E':
            region = 5;
            break;
        case 'F':
            region = 6;
            break;
        case 'G':
            region = 7;
            break;
        case 'H':
            region = 8;
            break;
    }
    
    

    //Waits until robot is positioned and button is pressed
    WaitForFinalAction();

    
    //Waits until the start light turns red
    while ((cds_cell.Value())>1.2)
    {
        Sleep(50);
    }

    //pushes button and moves to compost
    robot.move(3,30,10);
    robot.stopmot();
    robot.move(9,300,10);
    robot.turn(30,.5);
    robot.move(6,0,7);
    robot.move(.01,180,4);
    robot.move(4.5,270,10);

    //rotates the compost
    robot.stopmot();
    // robot.move(1,270,16);
    // robot.move(1,270,10);
    compost.SetDegree(180);
    Sleep(1.5);
    compost.Off();
    Sleep(10);
    compost.SetDegree(0);
    Sleep(1.5);

    compost.Off();
    robot.stopmot();
    //restes position and drives to apple bucket
    robot.move(4,270,7);
    




    robot.move(3.5,90,7);
    robot.move(4,0,5);
    if (region==7)
    {
        robot.move(8.5,180,10);
    }
    else
    {
        robot.move(8.3,180,10);
    }
    
    //apple bucket pickup and drive to window
    robot.armmove(39,.6);


    robot.stopmot();
    robot.turn(-162,1.25);
    
    robot.stopmot();
    robot.move(6.5,64,14);
    robot.stopmot();
    robot.armmove(0,.75);
    robot.turn(150 ,1.25);
    robot.stopmot();

    Sleep(.5);
    
    robot.move(10.5,191,12);
    
    
    int window = RCS.isWindowOpen();
    //open window and line up with window
    robot.move(12.5,90,12);
    robot.move(2.5,270,5);
    robot.move(8,330,14);
    robot.stopmot();
    robot.turn(-125,1);
    robot.move(30,180,12);
    robot.move(3,180,5);
    robot.move(2.4,0,7);
    robot.turn(-160,2);

    robot.move(47,59,17.25);
    robot.stopmot();
    Sleep(500);
    //Dropoff apple bucket
    robot.move(1,240,7);
    robot.stopmot();
    robot.armmove(20,.5);

    //Complete humidifier button press
    robot.move(2.1,235,10);
    robot.turn(180,1.25);
    robot.move(5.9,180,12);
    robot.armmove(0,.2);
    if (region==5)
    {
        robot.move(16.5,0,12);
    }
    else
    {
        robot.move(16.25,0,12);
    }
    LCD.WriteLine(cds_cell.Value());
    //Checks color of humidifer button indicator
     while (cds_cell.Value()>2.2)
    {
        Sleep(10);
    }
    if (cds_cell.Value()>1.6)
    {
        
        robot.move(8,15,12);
        robot.move(8,195,12);
        
    }
    else if(cds_cell.Value()<=1.6)
    {
        
        robot.move(8,345,12);
        robot.move(8,165,12);
        
    }
    robot.move(20,180,12);

   int lever= RCS.GetLever();
    robot.move(4,0,7);
    //checks which lever to press and presses said lever
    if (lever==1)
    {
        robot.turn(-105,2);
    }
    else if (lever==0)
    {
        robot.turn(-98,2);
    }
    else if (lever==2)
    {
        robot.turn(-107,2);
    }
    //move to lever
    robot.move(14.75,60,10);
    robot.armmove(0,.5);
    robot.stopmot();
    robot.armmove(70,.5);
    robot.armmove(0,.5);
    robot.move(1,240,5);
    robot.armmove(70,.6);
    robot.stopmot();
    Sleep(5.1);
    robot.move(4.5,60,5);
    robot.stopmot();
    robot.armmove(0,.5);
    Sleep(20);
    robot.armmove(40,.1);
    robot.move(14,240,10);
    robot.armmove(0,.1);
    //move back from lever and reset position
    robot.turn(105,1);
    robot.move(6,180,7);
    robot.move(9.5,0,10);
    //close window
    robot.turn(-90,.5);
    robot.move(5,180,5);
    robot.move(18.5,90,12);
    robot.move(2.5,270,7);
    robot.move(2.5,0,7);
    robot.turn(75,.5);
    robot.move(25,180,10);
    robot.move(3,0,8);
    robot.turn(90,1);
    //return to start and press final button
    robot.move(45,0,14);
    robot.move(4,180,7);
    robot.move(12,90,14);





    robot.stopmot();
    while (1)
    {

    }

   
   
}