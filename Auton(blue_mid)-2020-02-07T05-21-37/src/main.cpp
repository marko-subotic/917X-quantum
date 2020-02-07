#include "vex.h"

vex::motor Left = vex::motor(vex::PORT1); 
vex::motor Right = vex::motor(vex::PORT11, true); 

vex::motor IntakeLeft = vex::motor(vex::PORT2, true); 
vex::motor IntakeRight = vex::motor(vex::PORT12); 

vex::motor ArmAngleL = vex::motor(vex::PORT3); 
vex::motor ArmAngleR = vex::motor(vex::PORT13, true); 

vex::motor TrayAngle = vex::motor(vex::PORT4, true); 


//move (in tiles; + forward, - backwards)
void move(double dist, int spd)
{
    dist *=(343.77468*2);
    Left.startRotateFor(dist, vex::rotationUnits::deg, spd, vex::velocityUnits::pct);
    Right.rotateFor(dist, vex::rotationUnits::deg, spd, vex::velocityUnits::pct);
}

//turn degrees, (+ for right, - for left)
void turn(int deg)
{
    move(-0.5,60);
    deg *= 2.4;
    Left.startRotateFor(deg, vex::rotationUnits::deg, 70, vex::velocityUnits::pct);
    Right.rotateFor(-deg, vex::rotationUnits::deg, 70, vex::velocityUnits::pct);
}

//grab one cube and keep in robot, time in seconds, push for direction (+ for intake, - for pushing)
void grab(double time, int push)
{
    Left.startRotateFor(130*time, vex::rotationUnits::deg, 20, vex::velocityUnits::pct);
    Right.startRotateFor(130*time, vex::rotationUnits::deg, 20, vex::velocityUnits::pct);
    IntakeLeft.spin(vex::directionType::rev, 90*push, vex::velocityUnits::pct); 
    IntakeRight.spin(vex::directionType::rev, 90*push, vex::velocityUnits::pct);
    vex::task::sleep(time*1000);
    IntakeLeft.spin(vex::directionType::rev, 0, vex::velocityUnits::rpm); 
    IntakeRight.spin(vex::directionType::rev, 0, vex::velocityUnits::rpm); 
}

//take out cube, lift arm, place cube, and lower WIP
void tower()
{
    IntakeLeft.stop(); 
    IntakeRight.stop();
    
    IntakeLeft.startRotateFor(200, vex::rotationUnits::deg, 80, vex::velocityUnits::pct);
    IntakeRight.rotateFor(200, vex::rotationUnits::deg, 80, vex::velocityUnits::pct); 
    
    TrayAngle.rotateFor(40, vex::rotationUnits::deg, 50, vex::velocityUnits::pct);

    ArmAngleL.startRotateTo(240, vex::rotationUnits::deg, 30, vex::velocityUnits::pct); 
    ArmAngleR.rotateFor(240, vex::rotationUnits::deg, 30, vex::velocityUnits::pct); 
    vex::task::sleep(500);
    IntakeLeft.startRotateFor(180, vex::rotationUnits::deg, 90, vex::velocityUnits::pct);
    IntakeRight.rotateFor(180, vex::rotationUnits::deg, 90, vex::velocityUnits::pct); 
    vex::task::sleep(500);
    ArmAngleL.startRotateTo(55, vex::rotationUnits::deg); 
    ArmAngleR.rotateTo(55, vex::rotationUnits::deg); 
}

//slowly lower tray and score
void stack()
{
    IntakeLeft.startRotateFor(360, vex::rotationUnits::deg, -35, vex::velocityUnits::pct);
    IntakeRight.startRotateFor(360, vex::rotationUnits::deg, -35, vex::velocityUnits::pct);
    
    TrayAngle.rotateFor(-1200, vex::rotationUnits::deg, 60, vex::velocityUnits::pct);
    TrayAngle.rotateFor(-600, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
    vex::task::sleep(200);

    Left.startRotateFor(100, vex::rotationUnits::deg, 10, vex::velocityUnits::pct);
    Right.rotateFor(100, vex::rotationUnits::deg, 10, vex::velocityUnits::pct);
    vex::task::sleep(200);

    IntakeLeft.startRotateFor(360, vex::rotationUnits::deg, -35, vex::velocityUnits::pct);
    IntakeRight.startRotateFor(360, vex::rotationUnits::deg, -35, vex::velocityUnits::pct);
    vex::task::sleep(200);

    Left.startRotateFor(-360, vex::rotationUnits::deg, 50, vex::velocityUnits::pct);
    Right.rotateFor(-360, vex::rotationUnits::deg, 50, vex::velocityUnits::pct);
}

//stuff before auton
void start()
{
    move(0.2,50);
    ArmAngleL.startRotateTo(350, vex::rotationUnits::deg, 50, vex::velocityUnits::pct); 
    ArmAngleR.startRotateTo(350, vex::rotationUnits::deg, 50, vex::velocityUnits::pct);
    vex::task::sleep(300);
    TrayAngle.rotateTo(10, vex::rotationUnits::deg, 60, vex::velocityUnits::pct);
    vex::task::sleep(250);
    ArmAngleL.startRotateTo(25, vex::rotationUnits::deg); 
    ArmAngleR.rotateTo(5, vex::rotationUnits::deg); 
    TrayAngle.rotateTo(5, vex::rotationUnits::deg, 40, vex::velocityUnits::pct);
    move(-0.2,50);
    vex::task::sleep(700);
}

int main() 
{
    int s=60;
    //start();
    ArmAngleL.startRotateTo(30, vex::rotationUnits::deg); 
    ArmAngleR.rotateTo(30, vex::rotationUnits::deg); 
    //move(2,s);
    //grab(0.25,1);
    //stack();
    /*
    move(1,s );
    
    turn(90);
    move(0.5,s );
    tower();
    grab(1,1);
    move(-0.6,s );
    turn(-90);
    //*//*
    move(0.8,s );
    grab(2.5,1);
    turn(90);
    vex::task::sleep(200);
    move(-2.2,s );
    vex::task::sleep(500);
    
    turn(-90);
    move(1,s );
    turn(90);*/
    /*
    move(0.5,s );
    grab(5,1);
    move(-0.7,s);
    turn(-240);
    vex::task::sleep(400);*/
    //move(0.7,s);

    stack();
  //*/
  
  /*
    move(3);
    grab(5,1);
    move(-5);
    turn(-90);
    move(4);
    stack();
    vex::task::sleep(500);

  
    move(-1);
    turn(90);
    move(2.5);
    grab(5,1);
    move(-2);
    turn(90);
    move(9);
    turn(90);
    move(1);
    stack();
  */


}
