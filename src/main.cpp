#include <FEH.h>
#include <Arduino.h>
#include <stdio.h>
#include <math.h>

#define wheel_radius 1.25    //inches
#define robot_radius 3.639   // inches
#define wheel1_theta 0.0     // degrees
#define wheel2_theta 120.0     // degrees
#define wheel3_theta 240.0     // degrees
#define Pi 3.1415926535897
#define robot_weight 5         //kilograms
#define countsperinch 40.48    //counts per inch

#define motor_torque_weight 2


// Motor Defines
FEHMotor motor1(FEHMotor::Motor1, 9.0);
FEHMotor motor2(FEHMotor::Motor3, 9.0);
FEHMotor motor3(FEHMotor::Motor0, 9.0);
//Encoder Defines
DigitalEncoder encoder1(FEHIO::Pin8);
DigitalEncoder encoder2(FEHIO::Pin9);
DigitalEncoder encoder3(FEHIO::Pin10);




class robot{
    public:
    void move(float dist, float angle, float speed)
    {
        encoder1.ResetCounts();
        encoder2.ResetCounts();
        encoder3.ResetCounts();

        velocity_x=cos((angle/180)*Pi)*speed;
        velocity_y=sin((angle/180)*Pi)*speed;

        

        wheelspeedcalc(velocity_x,velocity_y, 0);

        encoder1dstcount=(dist/speed)*wheelspeed1*wheel_radius*countsperinch;
        encoder2dstcount=(dist/speed)*wheelspeed2*wheel_radius*countsperinch;
        encoder3dstcount=(dist/speed)*wheelspeed3*wheel_radius*countsperinch;


        while ((encoder1.Counts()<encoder1dstcount)||(encoder2.Counts()<encoder2dstcount)||(encoder3.Counts()<encoder3dstcount))
        {
            motor1.SetPercent(motor1_voltage);
            motor2.SetPercent(motor2_voltage);
            motor3.SetPercent(motor3_voltage);
        }
    
    }
    void wheelspeedcalc(float vx, float vy, float botrot)
    {
        
        wheelspeed1 = ((sin((wheel1_theta/360)*2*Pi)*vx+-1*cos((wheel1_theta/360)*2*Pi)*vy+(-1*robot_radius*botrot))/wheel_radius)*(60.0/(2.0*Pi));
        wheelspeed2 = ((sin((wheel2_theta/360)*2*Pi)*vx+-1*cos((wheel2_theta/360)*2*Pi)*vy+(-1*robot_radius*botrot))/wheel_radius)*(60.0/(2.0*Pi));
        wheelspeed3 = ((sin((wheel3_theta/360)*2*Pi)*vx+-1*cos((wheel3_theta/360)*2*Pi)*vy+(-1*robot_radius*botrot))/wheel_radius)*(60.0/(2.0*Pi));

        motor1_voltage = ((wheelspeed1/120)*100);
        motor2_voltage = ((wheelspeed2/120)*100);
        motor3_voltage = ((wheelspeed3/120)*100);

    }
    private:
    float wheelspeed1, wheelspeed2, wheelspeed3;
    float motor1_voltage, motor2_voltage, motor3_voltage;   //in percent
    float wheelspeed1, wheelspeed2, wheelspeed3;    // in rads/sec
    float encoder1dstcount, encoder2dstcount, encoder3dstcount;
    float velocity_x, velocity_y, rotation_rad;   // in inches per second and rotation of robot is in 
};







void ERCMain()
{

    
}