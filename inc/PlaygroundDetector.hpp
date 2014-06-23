#ifndef _PlaygroundDetector_H_
#define _PlaygroundDetector_H_

#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>
#include <vector>

#include "Playground.hpp"

typedef std::vector<std::vector<cv::Point> > Contours;


class PlaygroundDetector {


  public:
     
    PlaygroundDetector();
    ~PlaygroundDetector() {}

    /*
    * Find Playground in thres image and calc its Extrinsics
    */
    bool detect(const cv::Mat &thresImage, Playground &Playground, Contours &candidateContours, aruco::CameraParameters &cameraParameters);
        

  private:
    
    /*
    * Find all contours in thres image
    */
    void findContours(const cv::Mat &thresImage, Contours &contours) const;
    
    /*
    * Filter for L-shaped contours
    */
    void filterContours(const Contours &contours, Contours &filteredContours) const;
    
    /*
    * Combine exatly 4 L-contours to one rectangle with 4 corners
    */
    void extractPlayGroundCorners(const Contours &filteredContours, std::vector<cv::Point2f> &corners) const;
    

};

#endif
