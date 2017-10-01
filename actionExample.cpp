#include "Aria.h"
#include <ArRecurrentTask.h>
#include <string>
#include <iostream>
#include <thread>

using namespace std;

void goForward(ArRobot &robot, int distanceMM);
void goBackward(ArRobot &robot, int numBlocks);
void turnLeft(ArRobot &robot);
void turnRight(ArRobot &robot);
void turnBackwards(ArRobot &robot);
void stop(ArRobot &robot);

int main(int argc, char **argv) {
   // Init the robot and try to connect to it
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
  
   // Turn on the robot
   robot.enableMotors();
   robot.runAsync(true);
  
   // Add actions
   ArActionStallRecover recover;  
   robot.addAction(&recover, 50);

   // Set the initial position of the robot
   //ArPose posInicial = new ArPose(200,200,50);
   //robot.moveTo(ArPose(200,200,0));

   // Main program
   int TIMEOUT_REPEAT_LOOP_MS = 1000;
   bool foundPath = false;
   int count = 0;
   int currentHeading = 0;

   while (!foundPath && count < 7) {
      robot.lock();
      ArLog::log(ArLog::Normal, "New Iteraction ...");
      ArLog::log(ArLog::Normal, "GoalCoordinate : (%.2f,%.2f)", 400, 400);
      ArLog::log(ArLog::Normal, "Robo coordinate : (%.2f,%.2f,%.2f,Vel=%.2f)", robot.getX(), robot.getY(), robot.getTh(), robot.getVel());
      robot.unlock();

      switch(count) {
         
         case 0: {
            goForward(robot, 18);
            break; 
         }

         case 1: {
            turnLeft(robot);
            break; 
         }

         case 2: {
            goForward(robot, 9);

            // Hold this thread and then continue running
            //this_thread::sleep_for(std::chrono::milliseconds(TIMEOUT_REPEAT_LOOP_MS));
            break; 
         }

         case 3: {
            turnRight(robot);
            break; 
         }

         case 4: {
            goForward(robot, 2);
            break; 
         }

         case 5: {
            turnLeft(robot);
            break; 
         }

         case 6: {
            goForward(robot, 1);
            break; 
         }

      }

      count++;

      ArLog::log(ArLog::Normal, "End of iteraction : (%.2d)", count);
   }
   
   robot.stopRunning();
   robot.waitForRunExit();
  
   // Make it possible to exit
   Aria::exit(0);
   return 0;
}

/**
 * 1 block = 600mm/s * 5seconds
 * 1 block = robo's largest size
 */
void goForward(ArRobot &robot, int numBlocks) {
   const int BASE_BLOCK_VEL_MM_SEC = 600;
   const int BASE_BLOCK_TIME_MOTION_SEC = 1555;
  
   for(int i = 0; i < numBlocks; i++) {
      robot.lock();
      ArLog::log(ArLog::Normal, "Moving forward, block : (%.d/%.d)", i + 1, numBlocks);
      robot.unlock();

      robot.lock();
      robot.setRotVel(0);
      robot.setVel(BASE_BLOCK_VEL_MM_SEC);
      robot.unlock();
      ArUtil::sleep(BASE_BLOCK_TIME_MOTION_SEC);
      stop(robot);
   }
}

void goBackward(ArRobot &robot, int numBlocks) {
  const int BASE_BLOCK_VEL_MM_SEC = -600;
  const int BASE_BLOCK_TIME_MOTION_SEC = 1545;

  for(int i = 0; i < numBlocks; i++) {
     robot.lock();
     ArLog::log(ArLog::Normal, "Moving forward, block : (%.d/%.d)", i + 1, numBlocks);
     robot.unlock();

     robot.lock();
     robot.setRotVel(0);
     robot.setVel(BASE_BLOCK_VEL_MM_SEC);
     robot.unlock();
     ArUtil::sleep(BASE_BLOCK_TIME_MOTION_SEC);
     stop(robot);
  }
}

void turnLeft(ArRobot &robot) {
   robot.lock(); 
   robot.setRotVel(90);
   robot.unlock();
   ArUtil::sleep(1020);
   stop(robot);
}

void turnRight(ArRobot &robot) {
   robot.lock();
   robot.setRotVel(-90);
   robot.unlock();
   ArUtil::sleep(1030);
   stop(robot);
}

void turnBackwards(ArRobot &robot) {
  turnLeft(robot);
  turnLeft(robot);
}

void stop(ArRobot &robot) {
   robot.lock();
   robot.stop();
   robot.unlock();
   ArUtil::sleep(1500);
}
