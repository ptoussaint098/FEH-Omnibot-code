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
    void move(float dist, float angle, float speed)
    {
        encoder1.ResetCounts();
        encoder2.ResetCounts();
        encoder3.ResetCounts();

        velocity_x=cos((angle/180.0)*Pi)*speed;
        velocity_y=sin((angle/180.0)*Pi)*speed;

       
        wheelspeedcalc(velocity_x,velocity_y, 0);

        encoder1dstcount=((((fabs(wheelspeedrpm1)*wheel_radius*(2.0*Pi))/60)*(dist/speed))*countsperinch);
        encoder2dstcount=((((fabs(wheelspeedrpm2)*wheel_radius*(2.0*Pi))/60)*(dist/speed))*countsperinch);
        encoder3dstcount=((((fabs(wheelspeedrpm3)*wheel_radius*(2.0*Pi))/60)*(dist/speed))*countsperinch);

        if (encoder1dstcount==0)
        {
            encoder1dstcount+=15;
        }
        else if (encoder2dstcount==0)
        {
            encoder2dstcount+=15;
        }
        else if (encoder3dstcount==0)
        {
            encoder3dstcount+=15;
        }

       

        pidreset();

        while ((encoder1.Counts()<=encoder1dstcount)&&(encoder2.Counts()<=encoder2dstcount)&&(encoder3.Counts()<=encoder3dstcount))
        {
           
            motor1.SetPercent(motor1_voltage);
            motor2.SetPercent(motor2_voltage);
            motor3.SetPercent(motor3_voltage);


            Sleep(5);


            pidcalc();

           



            writefuncs();
           
        }

   
    }
    void wheelspeedcalc(float vx, float vy, float botrot)
    {
       
        wheelspeedrpm1 = ((sin((wheel1_theta/360.0)*2.0*Pi)*vx+-1.0*cos((wheel1_theta/360.0)*2.0*Pi)*vy+(-1.0*robot_radius*botrot))/wheel_radius)*(60.0/(2.0*Pi));
        wheelspeedrpm2 = ((sin((wheel2_theta/360.0)*2*Pi)*vx+-1.0*cos((wheel2_theta/360.0)*2.0*Pi)*vy+(-1.0*robot_radius*botrot))/wheel_radius)*(60.0/(2.0*Pi));
        wheelspeedrpm3 = ((sin((wheel3_theta/360.0)*2.0*Pi)*vx+-1.0*cos((wheel3_theta/360.0)*2.0*Pi)*vy+(-1.0*robot_radius*botrot))/wheel_radius)*(60.0/(2.0*Pi));


        motor1_voltage = ((wheelspeedrpm1/motormaxrpm)*100);
        motor2_voltage = ((wheelspeedrpm2/motormaxrpm)*100);
        motor3_voltage = ((wheelspeedrpm3/motormaxrpm)*100);


    }
    void pidcalc()
    {
        time_next_pid=millis();
        time_diff_pid=time_next_pid-last_time_pid;

        encoder1now=encoder1.Counts();
        encoder2now=encoder2.Counts();
        encoder3now=encoder3.Counts();

        actual_wheel_speed1=((encoder1now-encoder1last)/countsperinch)/(time_diff_pid*.001);
        actual_wheel_speed2=((encoder2now-encoder2last)/countsperinch)/(time_diff_pid*.001);
        actual_wheel_speed3=((encoder3now-encoder3last)/countsperinch)/(time_diff_pid*.001);

        actualwheelspeedrpm1=(actual_wheel_speed1*60.0)/(2*Pi);
        actualwheelspeedrpm2=(actual_wheel_speed2*60.0)/(2*Pi);
        actualwheelspeedrpm3=(actual_wheel_speed3*60.0)/(2*Pi);


        pid_error1=((fabs(wheelspeedrpm1)*Pi*wheel_radius*2.0)/60.0)-actual_wheel_speed1;
        pid_error2=((fabs(wheelspeedrpm2)*Pi*wheel_radius*2.0)/60.0)-actual_wheel_speed2;
        pid_error3=((fabs(wheelspeedrpm3)*Pi*wheel_radius*2.0)/60.0)-actual_wheel_speed3;

        pid_sumoferrors1+=pid_error1*(time_diff_pid*.001);
        pid_sumoferrors2+=pid_error2*(time_diff_pid*.001);
        pid_sumoferrors3+=pid_error3*(time_diff_pid*.001);

        pid_Pterm1= Pid_P_Constant*pid_error1;
        pid_Pterm2= Pid_P_Constant*pid_error2;
        pid_Pterm3= Pid_P_Constant*pid_error3;

        pid_Iterm1=Pid_I_Constant*pid_sumoferrors1;
        pid_Iterm2=Pid_I_Constant*pid_sumoferrors2;
        pid_Iterm3=Pid_I_Constant*pid_sumoferrors3;

        pid_Dterm1=Pid_D_Constant*((pid_error1-pid_lasterror1)/(time_diff_pid*.001));
        pid_Dterm2=Pid_D_Constant*((pid_error2-pid_lasterror2)/(time_diff_pid*.001));
        pid_Dterm3=Pid_D_Constant*((pid_error3-pid_lasterror3)/(time_diff_pid*.001));
       
        if (base_voltage1>=0)
        {
            motor1_voltage=pid_Pterm1+pid_Iterm1+pid_Dterm1+base_voltage1;
        }
        else
        {
            motor1_voltage=base_voltage1-pid_Pterm1-pid_Iterm1-pid_Dterm1;    
        }
        if (base_voltage2>=0)
        {
            motor2_voltage=pid_Pterm2+pid_Iterm2+pid_Dterm2+base_voltage2;
        }
        else
        {
            motor2_voltage=base_voltage2-pid_Pterm2-pid_Iterm2-pid_Dterm2;    
        }
        if (base_voltage3>=0)
        {
            motor3_voltage=pid_Pterm3+pid_Iterm3+pid_Dterm3+base_voltage3;
        }
        else
        {
            motor3_voltage=base_voltage3-pid_Pterm3-pid_Iterm3-pid_Dterm3;    
        }

        //setting current values as previous values for next run
        last_time_pid=time_next_pid;

        encoder1last=encoder1now;
        encoder2last=encoder2now;
        encoder3last=encoder3now;

        pid_lasterror1=pid_error1;
        pid_lasterror2=pid_error2;
        pid_lasterror3=pid_error3;

    }
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
    void turn(float degree, float time)
    {
        encoder1.ResetCounts();
        encoder2.ResetCounts();
        encoder3.ResetCounts();

        radpersec=((Pi*degree)/180)/time;

        wheelspeedcalc(0,0,radpersec);

        encoder1dstcount=((((fabs(wheelspeedrpm1)*wheel_radius*(2.0*Pi))/60)*(time))*countsperinch)-momentumfactor*3.85;
        encoder2dstcount=((((fabs(wheelspeedrpm2)*wheel_radius*(2.0*Pi))/60)*(time))*countsperinch)-momentumfactor*3.85;
        encoder3dstcount=((((fabs(wheelspeedrpm3)*wheel_radius*(2.0*Pi))/60)*(time))*countsperinch)-momentumfactor*3.85;

        if (encoder1dstcount<15)
        {
            encoder1dstcount+=15;
        }
        else if (encoder2dstcount<15)
        {
            encoder2dstcount+=15;
        }
        else if (encoder3dstcount<15)
        {
            encoder3dstcount+=15;
        }

        pidreset();

        while ((encoder1.Counts()<=encoder1dstcount)&&(encoder2.Counts()<=encoder2dstcount)&&(encoder3.Counts()<=encoder3dstcount))
        {
           
            motor1.SetPercent(motor1_voltage);
            motor2.SetPercent(motor2_voltage);
            motor3.SetPercent(motor3_voltage);

            Sleep(5);

            pidcalc();


           
           
            writefuncs();
           
        }
       

    }
    void armmove(float angle, float time_to_complete)
    {
        waitime=time_to_complete/(abs(angle-arm_angle));
        if (arm_angle<angle)
        {
            while (arm_angle<angle)
            {
                arm_angle+=1.0;
                arm.SetDegree(arm_angle);
                LCD.WriteLine(waitime);
               
                Sleep(waitime);
            }
           
        }
        else
        {
             while (arm_angle>angle)
            {
                arm_angle-=1.0;
                arm.SetDegree(arm_angle);
                Sleep(waitime);
                LCD.WriteLine(waitime);
            }

        }
       
    }
    void stopmot()
    {
        motor1.SetPercent(0.0);
        motor2.SetPercent(0.0);
        motor3.SetPercent(0.0);
    }
    //Purely a function for testing values
    void writefuncs()
    {
       
        FEHLog::printf("Desired Speed 1: %f\n", wheelspeedrpm1);
        FEHLog::printf("Actual Speed 1: %f\n", actualwheelspeedrpm1);
        FEHLog::printf("Desired Speed 2: %f\n", wheelspeedrpm2);
        FEHLog::printf("Actual Speed 2: %f\n", actualwheelspeedrpm2);
        FEHLog::printf("Desired Speed 3: %f\n", wheelspeedrpm3);
        FEHLog::printf("Actual Speed 3: %f\n", actualwheelspeedrpm3);
        FEHLog::printf("PID time: %f\n", time_diff_pid*.001);

    }
    void datatrack()
    {
        static FEHFile *motordatapntr = SD.FOpen("Motordata.txt", "w");
        SD.FPrintf(motordatapntr,"%f\n%f\n%f\n",motor1_voltage,motor2_voltage,motor3_voltage);

    }
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
};







void ERCMain()
{
   // RCS.InitializeTouchMenu("0910B7XJM");
    FEHLog::enableBLE(130);
    arm.SetMax(SERVO_MAX);
    arm.SetMin(SERVO_MIN);
    LCD.WriteLine("program started");
    robot robot;
   

    FEHFile *filepntr = SD.FOpen("Test.txt","w");
    SD.FPrintf(filepntr,"Test");
    FEHLog::printf("Test");
    SD.FCloseAll();

     while ((cds_cell.Value())>1.2);
    {
        Sleep(50);
    }


    robot.move(3,30,10);
    robot.stopmot();
    robot.move(16,306,6);
    robot.move(1,300,10);
    compost.SetDegree(100);
    Sleep(1.5);
    compost.Off();
    Sleep(10);
    compost.SetDegree(60);
    Sleep(1.5);

    compost.Off();

    robot.move(18,125,6);


    // robot.move(2,110,5);
    // robot.move(18,200,6);
    // robot.stopmot();
    // robot.turn(-149,1.25);
    // robot.stopmot();
    // robot.armmove(40,.75);
    // robot.move(6,65,6);
    // robot.stopmot();
    // robot.armmove(0,.75);
    // robot.turn(-140 ,1.25);


    // robot.move(30,0,8);
    // robot.stopmot();
    // robot.move(1,180,7);
    // robot.stopmot();
    // robot.stopmot();
    // robot.move(35,80,12);
    // robot.stopmot();
    // robot.move(2,270,5);
    // robot.stopmot();
    // robot.turn(35,1);
    // robot.stopmot();
    // robot.armmove(15,.75);
    // robot.move(8,240,5);
    // robot.stopmot();

    // //anything past this is theoretical

    // robot.turn(63,1);
    // robot.stopmot();
    // robot.armmove(0,.5);
    // robot.move(19.5,60,6);
    // robot.stopmot();
    // robot.armmove(70,1);
    // Sleep(50);
    // robot.armmove(0,1);
    // robot.move(3,240,6);
    // robot.armmove(70,1);
    // robot.move(7,60,6);
    // robot.stopmot();
    // Sleep(5000);
    // robot.move(1,240,6);
    // robot.stopmot();
    // robot.armmove(0,1);
    // robot.move(8,240,6);
    // robot.stopmot();
   


    // int Lever = RCS.GetLever();

    // if (Lever==0)
    // {

    // }
    // if (Lever==1)
    // {
       
    // }
    // if (Lever==2)
    // {
       
    // }





   

   
    while (1)
    {

    }

   
   
}

