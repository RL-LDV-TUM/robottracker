#ifndef _RobotTraffic_H_
#define _RobotTraffic_H_

#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>
#include <vector>
#include <map>
#include <ctime>
#include <mutex>
#include <atomic>

#include "PlaygroundDetector.hpp"

typedef std::pair < cv::Mat, std::time_t > RobotTrace;

class RobotTraffic : public aruco::Marker {


  public:
  
    RobotTraffic();
    ~RobotTraffic() {}
    
    /*
    * Update positions of robots (and playground)
    */
    void updatePositions(const std::vector<aruco::Marker> &markers, const Playground &playground);
    
    /*
    * Get robots last known position (RobotTrace)
    * by robot id
    */
    RobotTrace queryRobot(unsigned id);
    
    /*
    * Get stored Playground
    */
    Playground getPlayground() {return playGround;}

  protected:
    
    Playground playGround;
    cv::Mat pgPose_inv;
    std::map< int, RobotTrace > robotposes;
    
    std::mutex write_mutex;
    std::atomic_int reader_cnt;
    
    /*
    * Calc abs robot pose
    */
    cv::Mat calcPose(const aruco::Marker &marker) const;
    
    /*
    * calc inverse homog. 3D-Transformation mat
    */
    cv::Mat inverseTransformation(const cv::Mat &mat) const;
 
    /*
    * Ensure no query is in progress
    */
    void waitForZeroReaders();
};

#endif
