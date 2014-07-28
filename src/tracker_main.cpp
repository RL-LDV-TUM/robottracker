#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>

#include <aruco/aruco.h>
#include <opencv2/highgui/highgui.hpp>

#include "PlaygroundDetector.hpp"
#include "RobotTraffic.hpp"
#include "TrafficServer.hpp"
#include "HUD.hpp"

int main(int argc,char **argv)
{
    try
    {
        if (argc <= 1)
        {
          cerr<<"Invalid number of arguments:"<<endl;
          cerr<<"\tUsage: [CameraId] [Camera.yml] [markerlength(cm)] [serverport]"<<endl;
          return false;
        }
        
        /**************
        * Params
        ***************/

        unsigned cameraId = (argc > 1) ? std::atoi(argv[1]) : 0;
        std::string intrinsicFile = (argc > 2) ? argv[2] : "camera.yml";
        float markerSize = (argc > 3) ? std::atof(argv[3]) : 4.8f;
        unsigned serverPort = (argc > 4) ? std::atoi(argv[4]) : 5005;
        
        /**************
        * Vars
        ***************/
        cv::VideoCapture videoCapturer;
        cv::Mat inputImage, drawImage;
        aruco::CameraParameters cameraParameters;
        
        aruco::MarkerDetector mDetector;
        std::vector<aruco::Marker> markers;
       
        PlaygroundDetector pDetector;
        Playground playground(98.0f, 68.0f, 10.0f); // Real Playground
        Contours candidateContours;
        
        RobotTraffic robotTraffic;
        TrafficServer tserver(robotTraffic, serverPort);
        
        //read camera parameters if passed
        if (intrinsicFile =="")
        {
            std::cerr << "No camera calibration file given!" << std::endl;
            return -1;
        }
        
        cameraParameters.readFromXMLFile(intrinsicFile);
        
        cv::Size captureDimensions = cameraParameters.CamSize;
        
        /**************
        * Open Camera
        ***************/
        videoCapturer.open(cameraId);
        
        // dimensions
	      videoCapturer.set(CV_CAP_PROP_FRAME_WIDTH, captureDimensions.width);
	      videoCapturer.set(CV_CAP_PROP_FRAME_HEIGHT, captureDimensions.height);
	      
	      // frame rate
	      videoCapturer.set(CV_CAP_PROP_FPS, 12);
	      
	      if (!videoCapturer.isOpened())
	      {
            std::cerr << "Could not open camera" << std::endl;
            return -1;
        }
        
        //read first image to get the dimensions
        videoCapturer>>inputImage;
        
        if (inputImage.size() != captureDimensions)
        {
           std::cerr << "Camera dimension mismatch" << std::endl;
           return -1;
        }
        
        mDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);
        mDetector.setMinMaxSize(0.01, 0.5);
        mDetector.setThresholdParams(8, 6);
        
        /**************
        * Create GUI
        ***************/
        cv::namedWindow("image", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
        char key = 0;
        bool pause = false;
        bool lockPlayground = false;

        
        /**************
        * Run
        ***************/
        
        // server
        std::thread srvThread (&TrafficServer::run, &tserver);
        
        // vision
        while ( key!=27 && videoCapturer.grab())
        {
        
            if(!pause)
            {
            
              /**************
              * Detect
              ***************/
              
              // get image from camera
              videoCapturer.retrieve(inputImage);
              
              // clear
              markers.clear();
              candidateContours.clear();
              playground.id = -1;
              
                        // mute chatty library :-/
                        std::streambuf* cout_sbuf = std::cout.rdbuf(); // save original sbuf
                        std::ofstream   fout("/dev/null");
                        std::cout.rdbuf(fout.rdbuf()); // redirect 'cout' to a 'fout'
              
              // detect robot markers
              mDetector.detect(inputImage,markers,cameraParameters,markerSize, false);
              
                        // unmute original cout stream buffer
                        std::cout.rdbuf(cout_sbuf);
              
              // detect playground
              if(!lockPlayground)
                pDetector.detect(inputImage, playground, candidateContours, cameraParameters);

              
              // store positions of markers and playground
              robotTraffic.updatePositions(markers, playground);
            }
              
            /**************
            * Draw
            ***************/
            
            drawImage = inputImage.clone();
            
            HUD hud(drawImage);
            
            hud.drawPlayground(robotTraffic.getPlayground(), candidateContours, playground.isValid(), lockPlayground);
  
            hud.drawMarkers(markers);
            
            hud.drawLegend(pause, lockPlayground);
            
            // Show image
            cv::imshow("image", drawImage);
            
            // user interface ;-)
            switch(key)
            {
              case 100: // d: debug
                // do sth
              break;
              
              case 112: // p: pause
                pause = !pause;
              break;
              
              case 108: // l: lock playground
                lockPlayground = !lockPlayground;
              break;
            }
            
            //wait for key to be pressed
            key=cv::waitKey(10);
        
        }
        
        //  at the end: shutdown TrafficServer
        tserver.shutDown();
        srvThread.join();

    } catch (std::exception &ex)
    {
        std::cout<<"Exception :"<<ex.what()<<std::endl;
    }
}
