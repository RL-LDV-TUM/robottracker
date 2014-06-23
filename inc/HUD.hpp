#ifndef _HUD_H_
#define _HUD_H_

#include <opencv2/highgui/highgui.hpp>
#include <aruco/aruco.h>

#include "Playground.hpp"

class HUD {

  public:
  
    HUD(cv::Mat &image, cv::Point legendPos=(cv::Point(50, 50)) );
    ~HUD() {};
    
    /*
    * Draw playground, color according to lock / newly detected
    */
    void drawPlayground(const Playground &playground, const Contours &candidateContours, bool highlight = false, bool pglock = false); 
    
    /*
    * Draw all detected markers
    */
    void drawMarkers(const std::vector<aruco::Marker> &markers);
    
    /*
    * Show shortcuts and toggled flags
    */
    void drawLegend(bool pause, bool pglock);
    
  protected:
  
    cv::Mat &image;
    cv::Point legendPos;
    
};

#endif
