#include "RobotTraffic.hpp"

RobotTraffic::RobotTraffic() : pgPose_inv(cv::Mat::eye(4, 4, CV_32F))
{
 
}

/*
* Update positions of robots (and playground)
*/
void RobotTraffic::updatePositions(const std::vector<aruco::Marker> &markers, const Playground &playground)
{
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
      robotposes[robotMarker.id] = RobotTrace(calcPose(robotMarker), std::time(0));
      
      std::cout << "Found Marker: " << robotMarker.id << "|" << calcPose(robotMarker) << std::endl;
    }
  }
}

RobotTrace RobotTraffic::queryRobot(unsigned id)
{
  std::map< int, RobotTrace >::iterator trace;
  trace = robotposes.find(id);
  
  if(trace != robotposes.end())
    return trace->second;
  else
    return RobotTrace(cv::Mat::eye(4, 4, CV_32F), 0);

}

/*
* Calc robot pose depeding relative to playground
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
  
  // for playground we need absolute Pose_{camera}
  if(marker.id == 0) return absPose;
  
  // Pose_{playground} = PlaygroundPose_{camera}^-1 * Pose_{camera}
  cv::Mat relPose = pgPose_inv * absPose;
  
  return relPose;
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
