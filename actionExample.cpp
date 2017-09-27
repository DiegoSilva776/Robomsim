#include "Aria.h"
#include <ArRecurrentTask.h>
#include <string>
#include <iostream>
#include <thread>

using namespace std;

/**
 * Commands guide:

   Move forward
   robot.move(1000);

   Move backward
   robot.move(-1000);

   Turn right
   robot.setHeading(-90);
      
   Turn left
   robot.setHeading(90);

   Detect wall left
   ...

   Detect wall right
   ...

   Detect wall front
   ...

 */

int main(int argc, char **argv) {
   Aria::init();
   
   ArArgumentParser parser(&argc, argv);
   parser.loadDefaultArguments();
   ArRobot robot;
   ArRobotConnector robotConnector(&parser, &robot);
   
   if(!robotConnector.connectRobot()) {
      ArLog::log(ArLog::Terse, "simpleConnect: Could not connect to the robot.");
      
      if(parser.checkHelpAndWarnUnparsed()) {
         Aria::logOptions();
         Aria::exit(1);
      }
   }

   if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed()) {
      Aria::logOptions();
      Aria::exit(1);
   }
  
   ArLog::log(ArLog::Normal, "simpleConnect: Connected to robot.");
  
   robot.enableMotors();
   robot.runAsync(true);
  
   // Main program
   int TIMEOUT_REPEAT_LOOP_MS = 1000;
   bool foundPath = false;

   ArActionStallRecover recover;  
   robot.addAction(&recover, 50);

   // Set the initial position of the robot
   //ArPose posInicial = new ArPose(200,200,50);
   //robot.moveTo(ArPose(200,200,0));

   //robot.setVel(800);
   //robot.setRotVel(10);
 
   int count = 0;
   int currentHeading = 0;

   //while (!foundPath && count < 10) {
      robot.lock();
      ArLog::log(ArLog::Normal, "New Iteraction ...");
      ArLog::log(ArLog::Normal, "GoalCoordinate : (%.2f,%.2f)", 400, 400);
      ArLog::log(ArLog::Normal, "Robo coordinate : (%.2f,%.2f,%.2f,Vel=%.2f)", robot.getX(), robot.getY(), robot.getTh(), robot.getVel());
      robot.unlock();

      robot.move(5000);
      robot.setVel(1000);
      ArUtil::sleep(5000);
      
      robot.setRotVel(90);
      ArUtil::sleep(1500);

      robot.move(5000);
      robot.setVel(1000);
      ArUtil::sleep(5000);
      
      //this_thread::sleep_for(std::chrono::milliseconds(TIMEOUT_REPEAT_LOOP_MS));
      count++;

      ArLog::log(ArLog::Normal, "End of iteraction : (%.2d)", count);
   //}
  
   robot.stopRunning();
   robot.waitForRunExit();
  
   // Exit
   Aria::exit(0);
   return 0;
}
