#include "RobotTraffic.hpp"

#include <thread>
#include <chrono>

RobotTraffic::RobotTraffic() : pgPose_inv(cv::Mat::eye(4, 4, CV_32F)), reader_cnt(0)
{

}

/*
* Update positions of robots (and playground)
*/
void RobotTraffic::updatePositions(const std::vector<aruco::Marker> &markers, const Playground &playground)
{
  // lock data for reading threads
  write_mutex.lock();

  if(playground.isValid())
  {
    playGround = playground;
    cv::Mat pgPose = calcPose(playground);
    pgPose_inv = inverseTransformation(pgPose);
    robotposes[playground.id] = RobotTrace(pgPose, std::time(0));
    
    std::cout << "Found Playground: " << pgPose << std::endl;
  }
  
  if(playGround.isValid())
  {
    for( aruco::Marker robotMarker : markers)
    {
      if(robotMarker.id <= 0) continue;
      
      // calc Pose relative to Playground
      cv::Mat relPose = pgPose_inv * calcPose(robotMarker);
      
      robotposes[robotMarker.id] = RobotTrace(relPose, std::time(0));
      
      cv::Point2f pos(relPose.at<float>(0,3), relPose.at<float>(1,3));
      
      std::cout << "Found Marker: " << robotMarker.id << "|X:" << pos.x + playground.getWidth()*0.5f << "(" << pos.x << ") Y:" << pos.y + playground.getHeight()*0.5f << "(" << pos.y << ")" << std::endl;
    }
  }
  
  // free data lock for reading threads
  write_mutex.unlock();
}

/*
* Get robots last known position (RobotTrace)
* by robot id
*/
RobotTrace RobotTraffic::queryRobot(unsigned id)
{
  reader_cnt++;

  std::map< int, RobotTrace >::const_iterator trace;
  trace = robotposes.find(id);
  
  if(trace != robotposes.end())
    return trace->second;
  else
    return RobotTrace(cv::Mat::eye(4, 4, CV_32F), 0);
    
  reader_cnt--;

}

/*
* Calc abs robot pose
*/
cv::Mat RobotTraffic::calcPose(const aruco::Marker &marker) const
{
  
  cv::Mat R(3,3,CV_32FC1);
  cv::Rodrigues(marker.Rvec, R);
  
  
  cv::Mat absPose(4,4,CV_32FC1,cv::Scalar::all(0));

  // copy the Rotation matrix in the first 3*3 indeces
  R.copyTo(absPose(cv::Rect(0, 0, 3, 3)));

  // Translation
  absPose.at<float> (0,3) = marker.Tvec.at<float>(0,0);
  absPose.at<float> (1,3) = marker.Tvec.at<float>(1,0);
  absPose.at<float> (2,3) = marker.Tvec.at<float>(2,0);
  
  absPose.at<float>(3,3) = 1;
  
  return absPose;
}

/*
* calc inverse homog. 3D-Transformation mat
*/
cv::Mat RobotTraffic::inverseTransformation(const cv::Mat &mat) const
{
  cv::Mat inverse(4,4,CV_32FC1,cv::Scalar::all(0));
  
  cv::Mat R = mat(cv::Rect(0,0,3,3));
  R = R.t();
  R.copyTo(inverse(cv::Rect(0,0,3,3)));
  
  cv::Mat org = mat(cv::Rect(3,0,1,3));
  cv::Mat T = -R * org;
  T.copyTo(inverse(cv::Rect(3,0,1,3)));
  
  inverse.at<float>(3,3) = 1;
  
  return inverse;
}

/*
* Ensure no query is in progress
*/
void RobotTraffic::waitForZeroReaders()
{
  while(reader_cnt.load() > 0)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
}
