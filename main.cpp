/*
 * File:   main.cpp
 * Author: Pablo Munoz
 * Extension of controller written by Mark M. - Metrobotics
 *
 * Created on May 12, 2011, 10:15 AM
 */

#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include "OHCamera.h"
#include "definitions.h"

using namespace std;

void displayUsage(char** argv){
	cout << "USAGE: " << argv[0] << " [ options ]" << endl << endl;
	cout << "[ options ] " << endl ;
	cout << "\t-s <server_hostname>:<server_port>" << endl;
	cout << "\t-c <cameraNumber>" << endl;
	// exit(1);
	return;
}

void readConfigFile(ifstream& cfFile, string& csHost, int& csPort, int& camNum){
  string cmd, tmp;

  // parse configuration file + attempt to connect the player and central servers
  while( !cfFile.eof() ) {
    cmd = ""; 
    cfFile >> cmd ; 
    
    // if the line is commented skip it otherwise process.
    if (! (( cmd[0] == '/' && cmd[1] == '/' ) || ( cmd == "")) ){ 
      if ( cmd == "central_server" ) {
	cfFile >> csHost >> csPort ;
      }      
      else if ( cmd == "map" ){
	getline(cfFile, tmp); 
      }
      else if ( cmd == "robot" ){ 
	getline(cfFile, tmp); 
      }
      else if ( cmd == "camera" ){
	cfFile >> camNum; 
      }
      else {
	cout << "Unknown config command: " << cmd << endl;
	getline(cfFile, tmp); 
      }
    }
    else {
      // ignore the rest of the line.
      getline(cfFile, tmp); 
    }
  } 
}

int main(int argc, char **argv)
{
    //const char* optflags = "s:";
    int ch =0;
    int server_port = 0;// = 6667;
    string server_hostname = "";// = "localhost";
    int cameraNum = -1;
    //const char* filepath = "../../../config_files/robot.conf";
	const char* filepath = "../../config_files/robot.conf";
    ifstream configfile(filepath);
    
    if (!configfile){
      cout << "unable to open file: " << filepath << endl; 
      return 1 ; 
    }
    
    readConfigFile(configfile, server_hostname, server_port, cameraNum);

    /*    
    if(argc == 1){
        // Default behavior?
        displayUsage(argv);
        return 0;
    }
    else {
      while ((ch = getopt(argc, argv, "s:p:i:c:"))>0 ){
	cout << ch << optarg;
	switch(ch){
	case 's':
	  server_hostname = strtok(optarg, ":");
	  cout << "server_hostname: " << server_hostname << endl;
	  server_port = atoi(strtok(NULL, ":"));
	  cout << "server_port: " << server_port << endl;
	  break;
	case 'c':
	  cout << optarg << endl;
	  cameraNum = atoi(optarg);
	  cout << "camera: " << cameraNum << endl;
	  break;
	default:
	  displayUsage(argv);
	  return 1;
	  break;
	}
      }
    }
    */	
    // Fix this. It should read this from the command line.
    //cameraNum = 1;
	
    if(cameraNum == -1){
      displayUsage(argv);
      return 1;
    }
	
    
    //OHCamera cam(delete1, delete1, delete1, delete1);
    OHCamera cam(cameraNum);

	
    if(!cam.connect(server_hostname, server_port)){
        return 1;
    }
	cam.startCamera();
	
	//cam.cameraThread = new boost::thread(&OHCamera::imageLoop, &cam);
	cam.cameraThread = new boost::thread(&OHCamera::camBeat, &cam);
	cout << "Thread created" << endl;
	
    // Main Loop
    while (cam.getState() != STATE_QUIT) {
		cam.update();
		//usleep(1);
    }
	
    return 0;
}
