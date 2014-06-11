#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>

#include "PlaygroundDetector.hpp"
#include "RobotTraffic.hpp"
#include "TrafficServer.hpp"

int main(int argc,char **argv)
{
    try
    {
        if (argc<1)
        {
          cerr<<"Invalid number of arguments"<<endl;
          cerr<<"Usage: [CameraId] [Camera.yml] [markerlength(cm)]"<<endl;
          return false;
        }
        
        /**************
        * Params
        ***************/
        cv::Size captureDimensions(1280, 720);

        unsigned cameraId = (argc > 1) ? std::atoi(argv[1]) : 0;
        std::string intrinsicFile = (argc > 2) ? argv[2] : "camera.yml";
        float markerSize = (argc > 3) ? std::atof(argv[3]) : 4.8f;
        
        
        /**************
        * Vars
        ***************/
        aruco::MarkerDetector mDetector;
        cv::VideoCapture videoCapturer;
        std::vector<aruco::Marker> markers;
        cv::Mat inputImage;
        aruco::CameraParameters cameraParameters;
        RobotTraffic robotTraffic;
        
        TrafficServer tserver(robotTraffic);
        
        /**************
        * Open Camera
        ***************/
        videoCapturer.open(0);
        // dimensions
	      videoCapturer.set(CV_CAP_PROP_FRAME_WIDTH, captureDimensions.width);
	      videoCapturer.set(CV_CAP_PROP_FRAME_HEIGHT, captureDimensions.height);
	      
	      // frame rate
	      videoCapturer.set(CV_CAP_PROP_FPS, 12);
	      
	      if (!videoCapturer.isOpened())
	      {
            cerr<<"Could not open camera"<<endl;
            return -1;
        }
        
        //read first image to get the dimensions
        videoCapturer>>inputImage;
        
        if (inputImage.size() != captureDimensions)
        {
           cerr<<"Camera dimension mismatch"<<endl;
           return -1;
        }

        //read camera parameters if passed
        if (intrinsicFile!="")
        {
            cameraParameters.readFromXMLFile(intrinsicFile);
            cameraParameters.resize(inputImage.size());
        }
        
        mDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);
        mDetector.setMinMaxSize(0.01, 0.5);
        //mDetector.setThresholdParams(7, 7);
        
        /**************
        * Create GUI
        ***************/
        //cv::namedWindow("thres", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
        cv::namedWindow("image", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
        char key = 0;
        bool pause = false;
        bool lockPlayground = false;
        
        
        /**************
        * Run
        ***************/
        
        // server
        std::thread srvThread (&TrafficServer::run, tserver);
        
        // vision
        while ( key!=27 && videoCapturer.grab())
        {
        
            if(!pause)
            {
            
              videoCapturer.retrieve(inputImage);
              
                // mute chatty library :-/
                std::streambuf* cout_sbuf = std::cout.rdbuf(); // save original sbuf
                std::ofstream   fout("/dev/null");
                std::cout.rdbuf(fout.rdbuf()); // redirect 'cout' to a 'fout'
              
              mDetector.detect(inputImage,markers,cameraParameters,markerSize, false);
              
                // unmute original cout stream buffer
                std::cout.rdbuf(cout_sbuf);
              
              for (unsigned i=0;i<markers.size();i++)
              {
                  markers[i].draw(inputImage,cv::Scalar(0,0,255),1);
              }
              
              // playground
              PlaygroundDetector pDetector;
              //Playground playground(28.0f, 20.0f); // test playground: DIN A4 TODO!
              Playground playground(98.0f, 69.0f); // Real Playground
              
              if(!lockPlayground && pDetector.detect(mDetector.getThresholdedImage(), playground, inputImage) )
              {
                playground.calculateExtrinsics(cameraParameters);
                playground.draw(inputImage,cv::Scalar(0,0,255),4,cameraParameters);
              }
              
              // draw stored playground 
              if(robotTraffic.getPlayground().isValid())
              {
                cv::Scalar color = (lockPlayground) ? cv::Scalar(0,255,0) : cv::Scalar(0,134,209);
                robotTraffic.getPlayground().draw(inputImage,color,1,cameraParameters);
              }
              
              // draw playground lock notice
              if(lockPlayground)
              {
                Playground::drawLockedLabel(inputImage, cv::Point(50,50));
              }
              
              // store positions
              robotTraffic.updatePositions(markers, playground);

              
              //cv::imshow("thres",mDetector.getThresholdedImage());
              cv::imshow("image",inputImage);
              
            }
            
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
