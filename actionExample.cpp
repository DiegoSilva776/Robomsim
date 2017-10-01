/**
 * actionExample.cpp
 *
 * Authors: Diego M. Silva, diegomsilva.com
 * Licese: MIT
 */
#include "Aria.h"
#include <ArRecurrentTask.h>
#include <string>
#include <iostream>
#include <thread>

using namespace std;

/**
 * DATA
 */
 bool mHasAchievedGoal = false;
 float mLatestX = 0.0;
 float mLatestY = 0.0;
 float mLatestRot = 0.0;
 float mLatestSpeed = 0.0;

/**
 * FUNCTIONS' DECLARATION
 */ 
/**
 * Setters & Getters
 */
void setLatestX(float x);
float getLatestX();
void setLatestY(float y);
float getLatestY();
void setLatestRotation(float rot);
float getLatestRotation();
void setLatestSpeed(float speed);
float getLatestSpeed();
void setHasAchievedGoal(bool hasAchievedIt);
bool hasAchievedGoal();

/**
 * Motion Commands
 */
void goForward(ArRobot &robot, int distanceMM);
void goBackward(ArRobot &robot, int numBlocks);
void turnLeft(ArRobot &robot);
void turnRight(ArRobot &robot);
void turnBackwards(ArRobot &robot);
void stop(ArRobot &robot);

/**
 * Monitoring Commands
*/
void updateRobotCoordinatesRotationSpeed(ArRobot &robot);

/**
 * MAIN
 */
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
  
   // AddDevices
   // Sonar
   ArSonarDevice sonar;
   robot.addRangeDevice(&sonar);

   // Add actions
   ArActionStallRecover recover;
   robot.addAction(&recover, 100);

   // Turn the robot on
   robot.enableMotors();
   robot.runAsync(true);

   // Main program
   int TIMEOUT_REPEAT_LOOP_MS = 5000;
   bool foundPath = false;
   int count = 0;
   int currentHeading = 0;

   while (!foundPath && count < 7) {
      updateRobotCoordinatesRotationSpeed(robot);
      
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
            goForward(robot, 13);
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

      robot.lock();
      ArLog::log(ArLog::Normal, "End of iteraction : (%.2d)", count);
      robot.unlock();
   }
   
   robot.stopRunning();
   robot.waitForRunExit();
  
   // Make it possible to exit
   Aria::exit(0);
   return 0;
}

/**
 * FUNCTIONS' IMPLEMENTATION
 */
/**
 * Setters & Getters
 */
void setLatestX(float x) {
  mLatestX = x;
}

float getLatestX() {
   return mLatestX;
}

void setLatestY(float y) {
   mLatestY = y;
}

float getLatestY() {
   return mLatestY;
}

void setLatestRotation(float rot) {
   mLatestRot = rot;
}

float getLatestRotation() {
   return mLatestRot;
}

void setLatestSpeed(float speed) {
   mLatestSpeed = speed;
}

float getLatestSpeed() {
   return mLatestSpeed;
}

void setHasAchievedGoal(bool hasAchievedIt) {
   mHasAchievedGoal = hasAchievedIt;
}

bool hasAchievedGoal() {
   return mHasAchievedGoal;
}

/**
 * Motion Commands
 */
void goForward(ArRobot &robot, int numBlocks) {
   const int BASE_BLOCK_VEL_MM_SEC = 300;
   const int BASE_BLOCK_TIME_MOTION_SEC = 2400;
  
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
  const int BASE_BLOCK_VEL_MM_SEC = -300;
  const int BASE_BLOCK_TIME_MOTION_SEC = 2350;

  for(int i = 0; i < numBlocks; i++) {
     robot.lock();
     ArLog::log(ArLog::Normal, "Moving backward, block : (%.d/%.d)", i + 1, numBlocks);
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
   ArLog::log(ArLog::Normal, "Turning left ...");
   robot.unlock();

   robot.lock(); 
   robot.setRotVel(90);
   robot.unlock();
   ArUtil::sleep(1020);

   stop(robot);
}

void turnRight(ArRobot &robot) {
   robot.lock();
   ArLog::log(ArLog::Normal, "Turning right ...");
   robot.unlock();

   robot.lock();
   robot.setRotVel(-90);
   robot.unlock();
   ArUtil::sleep(1030);

   stop(robot);
}

void turnBackwards(ArRobot &robot) {
   robot.lock();
   ArLog::log(ArLog::Normal, "Turning backwards ...");
   robot.unlock();

   turnLeft(robot);
   turnLeft(robot);
}

void stop(ArRobot &robot) {
   robot.lock();
   robot.stop();
   robot.unlock();
   ArUtil::sleep(1500);

   updateRobotCoordinatesRotationSpeed(robot);
}

/**
 * Monitoring Commands
 */
void updateRobotCoordinatesRotationSpeed(ArRobot &robot) {
   robot.lock();
   // Log the current status
   ArLog::log(ArLog::Normal, ".....................................");
   ArLog::log(ArLog::Normal, "Robot status: X and Y  : [%.2f,%.2f]", robot.getX(), robot.getY());
   ArLog::log(ArLog::Normal, "Robot status: Rotation : [%.2f]", robot.getTh());
   ArLog::log(ArLog::Normal, "Robot status: Speed    : [%.2f]", robot.getVel());
   ArLog::log(ArLog::Normal, ".....................................");

   // Identify anomalies
   float MIN_MOVEMENT_RANGE_PER_BLOCK_MM = 100;

   if (abs(robot.getX() - getLatestX() < MIN_MOVEMENT_RANGE_PER_BLOCK_MM)) {
      ArLog::log(ArLog::Normal, "The robot found an obstacle at : [%.2f, %.2f]", robot.getX(), robot.getY());
      ArLog::log(ArLog::Normal, "The robot rotation is : [%.2f]", robot.getTh());
   }

   // Update the processing values
   setLatestX(robot.getX());
   setLatestY(robot.getY());
   setLatestRotation(robot.getTh());
   setLatestSpeed(robot.getVel());

   robot.unlock();
}
