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
	cout << "USAGE: " << argv[0] << " [config file]" << endl;
	//cout << "USAGE: " << argv[0] << " [ options ]" << endl << endl;
	//cout << "[ options ] " << endl ;
	//cout << "\t-s <server_hostname>:<server_port>" << endl;
	//cout << "\t-c <cameraNumber>" << endl;
	// exit(1);
	return;
}
struct ConfigData{
	int robotArea;
	int light;
	bool macTest;
};

void readConfigFile(ifstream& cfFile, string& csHost, int& csPort, int& camNum, string botNames[], CvPoint camData[], ConfigData & conf){
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
      else if ( cmd == "botId1" ){
		  cfFile >> botNames[0];
      }
      else if ( cmd == "botId2" ){
		  cfFile >> botNames[1];
      }
      else if ( cmd == "botId3" ){
		  cfFile >> botNames[2];
      }
	  else if ( cmd == "ptsPix" ){
		  cfFile >> camData[0].x >> camData[0].y >> camData[1].x >> camData[1].y >> camData[2].x >> camData[2].y >> camData[3].x >> camData[3].y >>
		  camData[4].x >> camData[4].y >> camData[5].x >> camData[5].y >> camData[6].x >> camData[6].y >> camData[7].x >> camData[7].y >>
		  camData[8].x >> camData[8].y;
		  ;
	  }
	  else if ( cmd == "ptsCm" ){
		  cfFile >> camData[9].x >> camData[9].y >> camData[10].x >> camData[10].y >> camData[11].x >> camData[11].y >> camData[12].x >> camData[12].y
			>> camData[13].x >> camData[13].y >> camData[14].x >> camData[14].y >> camData[15].x >> camData[15].y >> camData[16].x >> camData[16].y
			>> camData[17].x >> camData[17].y;					  
	  }
	  else if ( cmd == "botArea" ){
		  cfFile >> conf.robotArea;
	  }
	  else if ( cmd == "macTest" ){
		  conf.macTest = true;
	  }
	  else if ( cmd == "light" ){
		  cfFile >> conf.light;
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
void buttonClicked(){
}
struct data{
	OHCamera * camera;
//	GtkWidget * entry;
};
/*
static void enter_callback( GtkWidget *widget, data * userData)
{
	const gchar *entry_text;
	entry_text = gtk_entry_get_text (GTK_ENTRY (userData->entry));
	printf ("Entry contents: %s\n", entry_text);
	userData->camera->robotArea = 5000;
	
}
*/
void guiInit(int argc, char ** argv, OHCamera * cam){
	cout << "guiInit" << endl;
//	g_thread_init(NULL);
//	gdk_threads_init();
//	
//	GtkWidget *window;
//	GtkWidget *hBox;
//	GtkWidget *entry;
//	GtkWidget *button;
//	GtkWidget *xPosUL; 
//	GtkAdjustment *xPosULAdj;
//	gint tmp_pos;	
//  gdk_threads_enter();
//	gtk_init (&argc, &argv);

	/*
	window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
	gtk_widget_set_size_request(GTK_WIDGET (window), 200, 100);
	gtk_window_set_title (GTK_WINDOW (window), "Calibrator"); 
	gtk_widget_show  (window);
	g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);
	
	//struct data{
//		OHCamera camera;
//		GtkWidget entry;
//	};
	
	
	hBox = gtk_hbox_new (FALSE, 0);
	gtk_container_add(GTK_CONTAINER (window), hBox);
	gtk_widget_show(hBox);
	
	
	entry = gtk_entry_new ();
    gtk_entry_set_max_length (GTK_ENTRY (entry), 50);
	
	data userData;
	userData.camera = cam;
	userData.entry = entry;
	
	
    g_signal_connect (entry, "activate",
					  G_CALLBACK (enter_callback),
					  &userData);//entry);
    gtk_entry_set_text (GTK_ENTRY (entry), "hello");
    tmp_pos = GTK_ENTRY (entry)->text_length;
    gtk_editable_insert_text (GTK_EDITABLE (entry), " world", -1, &tmp_pos);
    gtk_editable_select_region (GTK_EDITABLE (entry),
								0, GTK_ENTRY (entry)->text_length);
    gtk_box_pack_start (GTK_BOX (hBox), entry, TRUE, TRUE, 0);
    gtk_widget_show (entry);
	
	xPosULAdj = (GtkAdjustment *) gtk_adjustment_new (50.0, 0.0, 100.0, 1.0, 5.0, 5.0);
	xPosUL = gtk_spin_button_new (xPosULAdj, 1.0, 0); 
	gtk_box_pack_start(GTK_BOX (hBox), xPosUL, TRUE, TRUE, 0);
	gtk_widget_show(xPosUL);	
	
	
	button = gtk_button_new_from_stock(GTK_STOCK_CLOSE);
	g_signal_connect_swapped(button, "clicked", G_CALLBACK(gtk_widget_destroy), window);
	gtk_box_pack_start (GTK_BOX (hBox), button, TRUE, TRUE, 0);
	gtk_widget_set_can_default (button, TRUE);
    gtk_widget_grab_default (button);
    gtk_widget_show (button);
	
	
	
	gtk_main ();
	gdk_threads_leave();
	 */
}

int main(int argc, char **argv)
{
    //const char* optflags = "s:";
    int ch =0;
    int server_port = 0;// = 6667;
    string server_hostname = "";// = "localhost";
    int cameraNum = -1;
    if (argc!= 2){
    	displayUsage(argv);
	return -1;
    }
    const char* filepath = argv[1];
    //const char* filepath = "../../../config_files/cam.conf";
    //const char* filepath = "../../config_files/cam.conf";
    //const char* filepath = "config_files/cam.conf";
    ifstream configfile(filepath);
    string botNames[3]; // TODO: Use MAXROBOTS from definitions    
	CvPoint camData[18];

    if (!configfile){
      cout << "unable to open file: " << filepath << endl; 
      return 1 ; 
    }
	
	ConfigData config; 
	// DEFAULT VALUES IF NO CONF. FILE IS PROVIDED 
	config.robotArea = 398;
	config.light = 170;
	config.macTest = false;
	
    readConfigFile(configfile, server_hostname, server_port, cameraNum, botNames, camData, config);

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
	
    OHCamera * cam = new OHCamera(cameraNum, botNames, camData);

	cam->robotArea = config.robotArea;
	cam->macTest = config.macTest;
	cam->binaryThresholdMin = config.light;
	
	
    if(!cam->connect(server_hostname, server_port)){
        return 1;
    }
	cam->startCamera();
	
	cam->cameraThread = new boost::thread(&OHCamera::camBeat, cam);
	cout << "Thread created" << endl;

    // Gui Control. This does not work in Ubuntu
    //boost::thread * guiThread = new boost::thread(guiInit, argc, argv, cam );
    	
    // Main Loop
    while (cam->getState() != STATE_QUIT) {
		cam->update();
		//usleep(1);
    }
    
    delete cam;
//    delete guiThread;    
	
    return 0;
}
