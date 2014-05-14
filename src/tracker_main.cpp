#include <iostream>
#include <fstream>
#include <sstream>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>

#include "PlaygroundDetector.hpp"
#include "RobotTraffic.hpp"

int main(int argc,char **argv)
{
    try
    {
        if (argc<1)
        {
          cerr<<"Invalid number of arguments"<<endl;
          cerr<<"Usage: [CameraId] [Camera.yml]"<<endl;
          return false;
        }
        
        /**************
        * Params
        ***************/
        cv::Size captureDimensions(1280, 720);

        unsigned cameraId = (argc > 1) ? std::atoi(argv[1]) : 0;
        std::string intrinsicFile = (argc > 2) ? argv[2] : "camera.yml";
        unsigned markerSize = 7; // TODO
        
        
        /**************
        * Vars
        ***************/
        aruco::MarkerDetector mDetector;
        cv::VideoCapture videoCapturer;
        std::vector<aruco::Marker> markers;
        cv::Mat inputImage;
        aruco::CameraParameters cameraParameters;
        RobotTraffic robotTraffic;
        
        
        
        /**************
        * Open Camera
        ***************/
        videoCapturer.open(0);
        // dimensions
	      videoCapturer.set(CV_CAP_PROP_FRAME_WIDTH, captureDimensions.width);
	      videoCapturer.set(CV_CAP_PROP_FRAME_HEIGHT, captureDimensions.height);
	      
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
        //mDetector.setThresholdParams(7, 7);
        
        /**************
        * Create GUI
        ***************/
        //cv::namedWindow("thres",1);
        cv::namedWindow("image",1);
        char key = 0;
        
        
        /**************
        * Run
        ***************/
        while ( key!=27 && videoCapturer.grab())
        {
            videoCapturer.retrieve(inputImage);
            
            mDetector.detect(inputImage,markers,cameraParameters,markerSize);
            
            for (unsigned i=0;i<markers.size();i++)
            {
                //cout<<markers[i]<<endl;
                markers[i].calculateExtrinsics(7.0f, cameraParameters, false);
                markers[i].draw(inputImage,cv::Scalar(0,0,255),1);
            }
            
            // playground
            PlaygroundDetector pDetector;
            Playground playground(28.0f, 20.0f); // DIN A4 TODO!
            
           if( pDetector.detect(mDetector.getThresholdedImage(), playground, inputImage) )
            {
              playground.calculateExtrinsics(cameraParameters);
              playground.draw(inputImage,cv::Scalar(0,0,255),1);
              //markers.push_back(playground);
            }
            
            // store positions
            robotTraffic.updatePositions(markers, playground);
            
            // key == d // for debug ;-)
            if(key == 100)
            {
              std::cout << "0:" << robotTraffic.queryRobot(0).first << std::endl;
              std::cout << "9:" << robotTraffic.queryRobot(9).first << std::endl;
            }
            
            //cv::imshow("thres",mDetector.getThresholdedImage());
            cv::imshow("image",inputImage);
            
            //wait for key to be pressed
            key=cv::waitKey(10);
        
        }   

    } catch (std::exception &ex)
    {
        cout<<"Exception :"<<ex.what()<<endl;
    }
}
