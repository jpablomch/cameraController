/* 
 * File:   OHCamera.h
 * Author: Pablo Munoz
 * * I am using Robot.cpp as a model. Thanks Mark for the contribution.
 * Created on May 12, 2011, 10:22 AM
 */

#ifndef OHCAMERA_H
#define	OHCAMERA_H

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "metrobotics.h"


#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include <sstream>
#include <cstring>
#include <math.h>


using namespace std;
class OHCamera {
 public:
  //OHCamera(string host, string dbname, string user, string pwd);
  OHCamera(int cam, string botsN[]);
  OHCamera(const OHCamera& orig);
  virtual ~OHCamera();
  int  getState() const { return mCurrentState;}
  void update();
  bool connect(const string& hostname, unsigned short port);
  bool startCamera();
  void disconnect();
  void init_state();
	
  boost::thread * cameraThread;

  // Multiple robots
  static const int MAXROBOTS = 3; // Move to definitions.
  struct botId{
	long sessionId;
	int posX;
  	int posY; 
  	double thetaR;
	bool sendCampose;
 	string name;
  };
  botId bots[MAXROBOTS];
  
  void camBeat(){
//		while(true){
//			//cout << "In thread " << endl;
//			//if(uniqueRobotIdTracking != -1 && sendCamposeApproved && ident_proc){
//			if(sendCamposeApproved && ident_proc){
//				do_state_action_campose_send();
//				sendCamposeApproved = false;
//				
//			}
//			usleep(500);
//		}
		// Multi-Robot
	  while(true){
		  for(int i=0; i<MAXROBOTS; i++){
			  if(bots[i].sendCampose && bots[i].sessionId != -1 && ident_proc){ // ident. Sorry no updates later like in the GUI. Maybe later.
				  do_state_action_campose_sendMulti(i);
				  bots[i].sendCampose = false;
			  }
		  }	
		  usleep(500);
	  }
  }
  
 private:
  int uniqueRobotIdTracking; // For now we are tracking only one robot. 
  bool ident_sent;
  bool ident_proc;
  bool endProgram; 
  bool sendCamposeApproved; // TODO: Change this
	
  
	
  
  int mCurrentState;
  int mPreviousState;
  string mNameID;
  long mSessionID;
  string mTypeID;
  
  
  bool skipFrame;
  int robotArea;
  int difAreaAr;
  int difAreaAb;
  int beatCircle;
  bool beatIncrease;
  
  // Pablo: I am using this for testing. Please fix to read from Conf file
  int realGridUpLeftX;
  int realGridUpLeftY;
  int realGridUpRightX;
  int realGridUpRightY;
  int realGridDownLeftX;
  int realGridDownLeftY;
  int realGridDownRightX; 
  int realGridDownRightY;
  int tempWidth;
  int tempHeigth;
  int mapHeight; 

  int camera;
  int binaryThresholdMin;
  int binaryThresholdMax;
  int upperLeftCornerX;
  int upperLeftCornerY;
  int upperRightCornerX;
  int upperRightCornerY;
  int lowerLeftCornerX;
  int lowerLeftCornerY;
  int lowerRightCornerX;
  int lowerRightCornerY;
  CvCapture* capture;
  
  IplImage* frame;
  IplImage* finalFrame;
  IplImage* gray_im;
  IplImage* binary_im;
  
  int drop; 
  
  int posX;
  int posY; 
  double thetaR;
  
  
  boost::asio::io_service mIOService;
  boost::asio::ip::tcp::socket mSocket;
  
  string mStringBuffer;
  
  static const double MAX_TIME_SILENCE = 60.0;
  static const double MAX_TIME_STATE   = 10.0;
  metrobotics::PosixTimer mSilenceTimer;
  metrobotics::PosixTimer mStateTimer;
  
  bool read(std::stringstream& ss);
  bool write(const std::stringstream& ss);
  bool msg_waiting() const;
  
  void do_state_change(int state);
  void do_state_action_init();
  void do_state_action_ack();
  void do_state_action_idle();
  void do_state_action_ping_send();
  void do_state_action_pong_read();
  void do_state_action_campose_send();
  void do_state_action_campose_sendMulti(int);
  void do_state_action_ident();
  void do_state_action_cmd_proc();
  
  IplImage* doPyrDown( IplImage* in, int filter = IPL_GAUSSIAN_5x5);
  int getLineLength(CvPoint* p0, CvPoint * p1);
  void getMidPoint(CvPoint* p0, CvPoint * p1, CvPoint &midPoint);
  void findShapes(IplImage* img, int robotArea, IplImage* ret);
  int getImage(int argc, char ** argv); // old main. 
  void imageLoop();
  void checkKey();
  void initId(botId&,string);	
  
	
  
};

#endif	/* OHCAMERA_H */

