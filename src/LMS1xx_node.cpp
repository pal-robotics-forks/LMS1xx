#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <LMS1xx/LMS1xx.h>

#include <limits>

#include <csignal>
#include <cstdio>

#define DEG2RAD M_PI/180.0
#define RAD2DEG 180.0/M_PI

int main(int argc, char **argv)
{
  // laser data
  LMS1xx laser;
  scanCfg cfg;
  scanDataCfg dataCfg;
  scanData data;
  // published data
  sensor_msgs::LaserScan scan_msg;
  // parameters
  std::string host;
  std::string frame_id;
  double range_max = 20.0;
  double start_angle = -135.0*DEG2RAD;
  double end_angle   =  135.0*DEG2RAD;
  double frequency   = 25.0;
  ros::init(argc, argv, "lms1xx");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");


  n.param<std::string>("host", host, "192.168.1.2");
  n.param<std::string>("frame_id", frame_id, "laser");
  n.param<double>("range_max", range_max, 20.0);
  n.param<double>("start_angle", start_angle,-135.0*DEG2RAD);
  n.param<double>("end_angle", end_angle,135.0*DEG2RAD);
  n.param<double>("frequency", frequency,25.0);

  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
  ROS_INFO("connecting to laser at : %s", host.c_str());

  // initialize hardware
  laser.connect(host);

  if (laser.isConnected())
  {
    ROS_INFO("Connected to laser.");

    laser.login();
    cfg = laser.getScanCfg();
    int num_values;
    if (cfg.angleResolution == 2500)
    {
      num_values = 1081;
    }
    else if (cfg.angleResolution == 5000)
    {
      num_values = 541;
    }
    else
    {
      ROS_ERROR("Unsupported resolution");
      return 0;
    }

    double sensor_start_angle = cfg.startAngle/10000.0 * DEG2RAD - M_PI/2.0;
    double sensor_end_angle   = cfg.stopAngle/10000.0 * DEG2RAD - M_PI/2.0;

    ROS_INFO_STREAM("\n\nLaser measurement config received from the sensor :");
    ROS_INFO("\t sensor start angle %.2f [deg] - %.6f [rad]", cfg.startAngle/10000.0 - 90, sensor_start_angle );
    ROS_INFO("\t sensor stop  angle %.2f [deg] - %.6f [rad]", cfg.stopAngle/10000.0  - 90, sensor_end_angle );
    ROS_INFO("\t sensor angle resolution %.2f [deg] - %.6f [rad]", cfg.angleResolution/10000.0,  cfg.angleResolution/10000.0 * DEG2RAD);
    ROS_INFO("\t sensor scan  frequency %.2f [hz]", cfg.scaningFrequency/100.0);
    ROS_INFO("\t sensor num   readings  %d", num_values);

    /// Setting fields of scan msg
    scan_msg.header.frame_id = frame_id;
    scan_msg.range_min = 0.01;
    scan_msg.range_max = range_max;

    int frequency_reducer = static_cast<int>( cfg.scaningFrequency/(100.0 * frequency) );
    frequency = cfg.scaningFrequency/(100.0* frequency_reducer);
    scan_msg.scan_time = 1.0/frequency;
    scan_msg.angle_increment = cfg.angleResolution/10000.0 * DEG2RAD;

    /// Swap start/end angle in case of inverted sign
    if(start_angle > end_angle)
    {
      double tmp = start_angle;
      start_angle = end_angle;
      end_angle = tmp;
    }

    /// Check if sensor starting angle is greater than requested starting angle
    if(start_angle < sensor_start_angle )
      start_angle = sensor_start_angle;

    /// Check if sensor ending angle is smaller than requested ending angle
    if(end_angle > sensor_end_angle)
      end_angle = sensor_end_angle;

    /// Adjusting min max angle values considering angular resolution
    start_angle = sensor_start_angle + scan_msg.angle_increment* (static_cast<int>( ( fabs(start_angle - sensor_start_angle)) /scan_msg.angle_increment ) + 1);
    end_angle   = sensor_end_angle - scan_msg.angle_increment* ( static_cast<int>( ( fabs(sensor_end_angle - end_angle)) /scan_msg.angle_increment ) + 1);

    /// Setting min max angle values in ros scan msg
    scan_msg.angle_min = start_angle;
    scan_msg.angle_max = end_angle;



    int filtered_num_values = static_cast<int>((end_angle - start_angle)/scan_msg.angle_increment ) + 1;
    //    ROS_INFO("Sensor num of readings %d, filtered num of readings %d", num_values, filtered_num_values);
    num_values = filtered_num_values;

    ROS_INFO_STREAM("\n\nLaser measurement output from the driver :");

    ROS_INFO("\t msg start angle %.2f [deg] - %.6f [rad]", start_angle*RAD2DEG, start_angle);
    ROS_INFO("\t msg stop  angle %.2f [deg - %.6f [rad]", end_angle*RAD2DEG, end_angle);
    ROS_INFO("\t msg angle resolution %.2f [deg] - %.6f [rad]", cfg.angleResolution/10000.0, scan_msg.angle_increment);
    ROS_INFO("\t msg topic frequency %.2f [hz]", frequency);
    ROS_INFO("\t msg min   range %.2f [m]", scan_msg.range_min);
    ROS_INFO("\t msg max   range %.2f [m]", scan_msg.range_max);
    ROS_INFO("\t msg num   readings %d", num_values);

    ROS_INFO("\t frequency reduction factor is %d", frequency_reducer);

    scan_msg.time_increment = scan_msg.scan_time/num_values;
    scan_msg.ranges.resize(num_values);
    scan_msg.intensities.resize(num_values);

    dataCfg.outputChannel = 1;
    dataCfg.remission = true;
    dataCfg.resolution = 1;
    dataCfg.encoder = 0;
    dataCfg.position = false;
    dataCfg.deviceName = false;
    dataCfg.outputInterval = 1;

    laser.setScanDataCfg(dataCfg);

    laser.startMeas();

    status_t stat;
    do // wait for ready status
    {
      stat = laser.queryStatus();
      ros::Duration(1.0).sleep();
    }
    while (stat != ready_for_measurement);

    laser.startDevice(); // Log out to properly re-enable system after config

    laser.scanContinous(1);

    int counter = -1;

    while (ros::ok())
    {
      ros::Time start = ros::Time::now();

      scan_msg.header.stamp = start;
      ++scan_msg.header.seq;

      laser.getData(data);

      /// frequency publishing reduction
      counter++;
      if(counter%frequency_reducer != 0)
        continue;

      double angle = sensor_start_angle;

      int j = 0;
      for (int i = 0; i < data.dist_len1; i++)
      {
        if( angle - scan_msg.angle_min < -1e-4 || angle - scan_msg.angle_max > 1e-4 )
        {
          angle += scan_msg.angle_increment;
          continue;
        }
        if( data.dist1[i] == 0.0 )
          scan_msg.ranges[j] = std::numeric_limits<float>::infinity();
        else
          scan_msg.ranges[j] = data.dist1[i] * 0.001;
        angle += scan_msg.angle_increment;
        j++;
      }

      angle = sensor_start_angle;
      j = 0;
      for (int i = 0; i < data.rssi_len1; i++)
      {
        if(angle - scan_msg.angle_min < -1e-4 || angle - scan_msg.angle_max > 1e-4)
        {
          angle += scan_msg.angle_increment;
          continue;
        }
        scan_msg.intensities[j] = data.rssi1[i];
        angle += scan_msg.angle_increment;
        j++;
      }

      scan_pub.publish(scan_msg);

      ros::spinOnce();
    }

    laser.scanContinous(0);
    laser.stopMeas();
    laser.disconnect();
  }
  else
  {
    ROS_ERROR("Connection to device failed");
  }
  return 0;
}
