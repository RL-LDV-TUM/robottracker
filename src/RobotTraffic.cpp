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
* Get robots last known position (RobotMsg)
* by robot id
*/
RobotMsg RobotTraffic::queryRobot(int id)
{
  reader_cnt++;

  std::map< int, RobotTrace >::const_iterator trace_it;
  trace_it = robotposes.find(id);
  
  RobotMsg msg;
  
  // not found 
  if(trace_it == robotposes.end())
  {
    msg.id = 0;
    msg.sec = -1;
    msg.x = 0.0f;
    msg.y = 0.0f;
    msg.z = 0.0f;
    msg.xCell = 0;
    msg.yCell = 0;
    msg.angle = 0.0f;
    
  // found
  } else {
  
    const RobotTrace trace = trace_it->second;
    const cv::Mat &pose = trace.first;
    const cv::Point cell = calcCell(pose);
      
    msg.id = id;
    msg.sec = difftime(std::time(0), trace.second);
    msg.x = pose.at<float>(0,3) + playGround.getWidth()*0.5f;
    msg.y = pose.at<float>(1,3) + playGround.getWidth()*0.5f;
    msg.z = pose.at<float>(2,3);
    msg.xCell = cell.x;
    msg.yCell = cell.y;
    msg.angle = calcAngle(pose);
    
  }
  
  reader_cnt--;
  
  return msg;

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
* Calc rounded cell from coordinates
*/
cv::Point RobotTraffic::calcCell(const cv::Mat &pose)
{
  
  float x = pose.at<float> (0,3) + playGround.getWidth()*0.5f;
  float y = pose.at<float> (1,3) + playGround.getWidth()*0.5f;
  x = std::min(std::max(x, 0.0f), playGround.getWidth());
  y = std::min(std::max(y, 0.0f), playGround.getHeight());
std::cout << x << "|" << y << std::endl;
  return cv::Point(x/playGround.getCellLength(), y/playGround.getCellLength());
}

/*
* Calc robot rotation angle from pose
*/
float RobotTraffic::calcAngle(const cv::Mat &pose)
{
  float r21 = pose.at<float> (1,0);
  float r11 = pose.at<float> (0,0);
  return atan2(r21, r11);
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
