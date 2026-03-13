#include <FEH.h>
#include <Arduino.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "FEHServo.h"


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
#define Pid_P_Constant  .75
#define Pid_D_Constant  .1
#define Pid_I_Constant  1.3



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

        encoder1dstcount=((((fabs(wheelspeedrpm1)*wheel_radius*(2.0*Pi))/60)*(dist/speed))*countsperinch)-momentumfactor;
        encoder2dstcount=((((fabs(wheelspeedrpm2)*wheel_radius*(2.0*Pi))/60)*(dist/speed))*countsperinch)-momentumfactor;
        encoder3dstcount=((((fabs(wheelspeedrpm3)*wheel_radius*(2.0*Pi))/60)*(dist/speed))*countsperinch)-momentumfactor;
       

        pidreset();

        while ((encoder1.Counts()<=encoder1dstcount)||(encoder2.Counts()<=encoder2dstcount)||(encoder3.Counts()<=encoder3dstcount))
        {
            LCD.Clear();
            motor1.SetPercent(motor1_voltage);
            motor2.SetPercent(motor2_voltage);
            motor3.SetPercent(motor3_voltage);


            pidcalc();
           
            writefuncs();
            Sleep(10);
        }
        motor1.SetPercent(0);
        motor2.SetPercent(0);
        motor3.SetPercent(0);

   
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

        pidreset();

        while ((encoder1.Counts()<=encoder1dstcount)||(encoder2.Counts()<=encoder2dstcount)||(encoder3.Counts()<=encoder3dstcount))
        {
            LCD.Clear();
            motor1.SetPercent(motor1_voltage);
            motor2.SetPercent(motor2_voltage);
            motor3.SetPercent(motor3_voltage);


            pidcalc();
           
            writefuncs();
            Sleep(10);
        }
        motor1.SetPercent(0);
        motor2.SetPercent(0);
        motor3.SetPercent(0);

    }
    //Purely a function for testing values
    void writefuncs()
    {
        LCD.WriteLine(motor1_voltage);
        LCD.WriteLine(motor2_voltage);
        LCD.WriteLine(motor3_voltage);
        LCD.WriteLine(encoder1last);
        LCD.WriteLine(encoder2last);
        LCD.WriteLine(encoder3last);

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
};







void ERCMain()
{
    LCD.WriteLine("program started");
    robot robot;
   
    while ((cds_cell.Value())>1.2);
    {
        Sleep(50);
    }
    robot.move(2,300,10);
    robot.move(1,75,11);
    robot.turn(-83.5,.75);
    robot.move(36,180,11);
    robot.move(10,180,5);
    robot.move(4.75,0,6);
    robot.turn(-80, .75);
   
    robot.move(6,180,5);
    robot.move(15.75,0,11);


    robot.move(7,45,7);
    while (!RCS.isWindowOpen())
    {
        robot.move(1,180,5);
    }
    robot.move(2,270,7);

    


    while (cds_cell.Value()>2.2)
    {
        Sleep(10);
    }
    if (cds_cell.Value()>1.4)
    {
        LCD.WriteLine("blue");
        LCD.WriteLine(cds_cell.Value());
        Sleep(10.0);
        robot.move(5.5,20,6);
        robot.move(6,200,7);
        robot.move(18,180,7);
        robot.move(4,0,5);
        robot.move(25,90,10);
        robot.move(4,180,5);
        robot.move(4,0,5);
        robot.move(13,90,10);
    }
    else if(cds_cell.Value()<=1.4)
    {
        LCD.WriteLine("Red");
        LCD.WriteLine(cds_cell.Value());
        Sleep(10.0);
        robot.move(5.5,340,6);
        robot.move(6,160,7);
        robot.move(18,180,7);
        robot.move(4,0,5);
        robot.move(25,90,10);
        robot.move(4,180,5);
        robot.move(4,0,5);
        robot.move(13,90,10);
    }



    while (1)
    {

    }

   
   
}

