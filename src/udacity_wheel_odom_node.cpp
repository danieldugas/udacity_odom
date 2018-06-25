#include <string>
#include <cmath>
#include <algorithm>

#include <glog/logging.h>
#include <ros/ros.h>
#include <dbw_mkz_msgs/WheelSpeedReport.h>
#include <tf/transform_broadcaster.h>

namespace udacity_wheel_odom {

class WheelOdomTracker {

  public:
    explicit WheelOdomTracker(ros::NodeHandle& n) : nh_(n) {
      // Topic names.
      const std::string kWheelSpeedReportTopic = "/vehicle/wheel_speed_report";

      // Publishers and subscribers.
      wheel_speed_sub_ = nh_.subscribe(kWheelSpeedReportTopic, 100, &WheelOdomTracker::wheelSpeedCallback, this);
    }
    ~WheelOdomTracker() {}

  protected:
    /// \brief receives joystick messages
    void wheelSpeedCallback(const dbw_mkz_msgs::WheelSpeedReport::ConstPtr& msg) {
      // Frame constants.
      const std::string kBaseFrameName = "world";
      const std::string kOdomFrameName = "odom";
      // Hardware constants.
      const static float kWheelRadiusMm = ( 678. / 2. );
      const static float kWheelbaseMm = 2728.;
      const static float kRotationCorrection = 1.6; // ugly empirical param to make it work.
      // 'forward' direction is y coordinate in base_link frame.

      if ( is_called_for_the_first_time_ ) {
        is_called_for_the_first_time_ = false;
        // initialize odom estimate.
        current_x_estimate_ = 0.;
        current_y_estimate_ = 0.;
        current_theta_estimate_ = 0.;

        // initialize values.
        prev_wheelspeeds_ = *msg;
        return;
      }

      // Interpolate the speed between old and new.
      float interp_rear_left_rads = ( prev_wheelspeeds_.rear_left + msg->rear_left ) / 2.;
      float interp_rear_right_rads = ( prev_wheelspeeds_.rear_right + msg->rear_right ) / 2.;
      double groundspeed_rear_left_mms = interp_rear_left_rads * kWheelRadiusMm;
      double groundspeed_rear_right_mms = interp_rear_right_rads * kWheelRadiusMm;
      double groundspeed_middle_mms = ( groundspeed_rear_left_mms + groundspeed_rear_right_mms ) / 2.;

      // Calculate dt
      unsigned long dt_ns = ( msg->header.stamp - prev_wheelspeeds_.header.stamp ).toNSec();

      // Assuming small rotation in 1/100th of second,
      // translation is moving forward along the previous heading.
      double forward_movement_m = groundspeed_middle_mms * (double)dt_ns * 1e-12;
      double y_movement_m = forward_movement_m *  cos( current_theta_estimate_ );
      double x_movement_m = forward_movement_m * -sin( current_theta_estimate_ );
      current_x_estimate_ += x_movement_m;
      current_y_estimate_ += y_movement_m;

      // Rotation.
      double dtheta = kRotationCorrection * 1e-9 * (double)dt_ns * ( groundspeed_rear_right_mms - groundspeed_rear_left_mms ) / kWheelbaseMm;
      current_theta_estimate_ += dtheta;

      // Publish tf.
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(current_x_estimate_, current_y_estimate_, 0.0) );
      tf::Quaternion quaternion;
      quaternion.setRPY(0., 0., current_theta_estimate_);
      transform.setRotation(quaternion);
      br_.sendTransform(
          tf::StampedTransform(transform, msg->header.stamp, kBaseFrameName, kOdomFrameName)
      );

      // Update values for next loop.
      prev_wheelspeeds_ = *msg;
    }

  private:
    ros::NodeHandle& nh_;
    ros::Subscriber wheel_speed_sub_;
    tf::TransformBroadcaster br_;
    bool is_called_for_the_first_time_ = true;
    dbw_mkz_msgs::WheelSpeedReport prev_wheelspeeds_;
    double current_x_estimate_;
    double current_y_estimate_;
    double current_theta_estimate_;
    
}; // class WheelOdomTracker

} // namespace udacity_wheel_odom

using namespace udacity_wheel_odom;

int main(int argc, char **argv) {

  ros::init(argc, argv, "udacity_wheel_odom");
  ros::NodeHandle n;
  WheelOdomTracker wheel_odom_tracker(n);

  try {
    ros::spin();
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception: " << e.what());
    return 1;
  }
  catch (...) {
    ROS_ERROR_STREAM("Unknown Exception.");
    return 1;
  }

  return 0;
}

