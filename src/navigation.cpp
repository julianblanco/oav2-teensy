//*****************************************************************
// Navigation Thread 1HZ
//*****************************************************************
#include "navigation.h"
 #include "config.h"
NAVIGATION::NAVIGATION() {}
NAVIGATION::~NAVIGATION() {}

int NAVIGATION::setup()
{
//
#ifdef MPU6050
  mpu6050init();
#endif
#ifdef BNO055
//shit
#endif
  Task::setup("navigation", 6);
  return 0;
}

int NAVIGATION::start()
{
  while (1)
  {

    LOOPFREQ(100);//hz
  }
}

 void NAVIGATION::NEDpositionControl()
{
  float northError = desiredNorth -currentNorth;
  float eastError = currentEast - desiredEast;
  float downError = currentDown - desiredDown;

  float northResponse = 0;
  float eastResponse = 0;
  float downResponse = 0;

  float pNorthResponse = 0;
  float pEastResponse = 0;
  float pDownResponse = 0;

  float iNorthResponse = 0;
  float iEastResponse = 0;
  float iDownResponse = 0;

  float dNorthResponse = 0;
  float dEastResponse = 0;
  float dDownResponse = 0;

  float northErrorLast = 0;
  float eastErrorLast = 0;
  float downErrorLast = 0;
  float mixer = 0;
  while (1)
  {
    if (g_armed)
    {
      if (g_current_mode == 2)
      {
        northError = g_navigation.currentNorth - g_navigation.desiredNorth;
        eastError = g_navigation.currentEast - g_navigation.desiredEast;
        downError = g_navigation.currentDown - g_navigation.desiredDown;

        pNorthResponse = northError * northKp;
        pEastResponse = eastError * eastKp;
        pDownResponse = downError * downKp;

        iNorthResponse += northError * northKi;
        iEastResponse += eastError * eastKi;
        iDownResponse += downError * downKi;

        dNorthResponse = northKd * (northError - northErrorLast) / 10;
        dEastResponse = eastKd * (eastError - eastErrorLast) / 10;
        dDownResponse = downKd * (downError - downErrorLast) / 10;

        northResponse = pNorthResponse + iNorthResponse + dNorthResponse;
        eastResponse = pEastResponse + iEastResponse + dEastResponse;
        downResponse = pDownResponse + iDownResponse + dDownResponse;

        g_navigation.desiredPitch = saturate(northResponse, 30, -30);
        g_navigation.desiredRoll = saturate(eastResponse, 30, -30);
        g_navigation.desiredThrottle = saturate(downResponse, 1000, -1000);
      }
      if (g_current_mode == 1)
      {
        //handled elsewhere
      }
    }

    //
  }
}
void NAVIGATION::headingFromGPS()
{
  int waypointNumber = 0;
  int numOfWaypoints = 5; //size of waypoints
  float oldHeading = 0;
  float courseBetweenWaypoints = 0;
  float distanceBetweenWaypoints = 0;
  float targetHeading = 0;
  float lastdesiredLat = 0;
  float lastdesiredLong = 0;
  while (1)
  {
    //calculates the distance to the waypoint
    float distanceToTarget = distanceToWaypoint(g_sensors.gps.latitudeDegrees, g_sensors.gps.longitudeDegrees, desiredLat,  desiredLong);

    if (flag_are_waypointing == 1)
    {
      float XTerror = crossTrackError(distanceToTarget, courseBetweenWaypoints, oldHeading);
      if (waypointNumber > 1 && XTerror > 1)
      {
        oldHeading = courseToWaypoint(g_sensors.gps.latitudeDegrees, g_sensors.gps.longitudeDegrees, desiredLat,  desiredLong);
        targetHeading = crossTrackCorrection(XTerror, oldHeading, distanceToTarget);
      }
      else
      {
        targetHeading = courseToWaypoint(g_sensors.gps.latitudeDegrees, g_sensors.gps.longitudeDegrees, g_navigation.desiredLat,  g_navigation.desiredLong);
      }

      if (waypointNumber > 1)
      {
        lastdesiredLat = wplat[waypointNumber - 1];
        lastdesiredLong = wplong[waypointNumber - 1];
      }

      if (distanceToTarget < waypointmindistance)
      {
        waypointNumber = waypointNumber + 1;

        if (waypointNumber == numOfWaypoints)
        {
          waypointNumber = 1;
        }
      }

      desiredLat = wplat[waypointNumber];
      desiredLong = wplong[waypointNumber];

      distanceBetweenWaypoints = distanceToWaypoint(desiredLat, desiredLong, wplat[waypointNumber - 1], wplong[waypointNumber - 1]);
      courseBetweenWaypoints = courseToWaypoint(wplat[waypointNumber - 1], wplong[waypointNumber - 1], desiredLat, desiredLong);
    }
  }
}

float  NAVIGATION::crossTrackCorrection(float distanceXT, float targetHead, float distance2WP)
{
  float xtCoeff = -100; // based on experimental data from the autonomous car
  float temp = (xtCoeff * distanceXT) / distance2WP;

  if (temp > 30)
    temp = 30; // maximum allowable correction
  if (temp < -30)
    temp = -30;

  float newTargetHeading = targetHead + temp;

  if (newTargetHeading >= 360)
    newTargetHeading -= 360;
  else if (newTargetHeading < 0)
    newTargetHeading += 360;

  return newTargetHeading;
} // end crossTrackError

float NAVIGATION::crossTrackError(float distance2WP, float tracklegHead, float targetHead)
{
  //convert to radians for use with sin
  tracklegHead = (3.14159265 / 180) * tracklegHead;
  targetHead = (3.14159265 / 180) * targetHead;

  //compute heading error off trackline
  float deltaHeading = tracklegHead - targetHead;

  // crosstrack distance (positive if right of track)
  float distanceXT = distance2WP * sin(deltaHeading);

  return distanceXT;
}


float NAVIGATION::distanceToWaypoint(float Lat1, float Long1, float Lat2, float Long2)
{
  float dist;
  float dLat = (float)(Lat2 - Lat1);               // difference of latitude in 1/10 000 000 degrees
  float dLon = (float)(Long2 - Long1) * cos(Lat1); //
  dist = sqrt(sq(dLat) + sq(dLon)) * 110312;

  return dist;
}

float NAVIGATION::courseToWaypoint(float lat1, float long1, float lat2, float long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  float dlon = radians(long2 - long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float a1 = sin(dlon) * cos(lat2);
  float a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
} // courseToWaypoint()

void NAVIGATION::convertToNED(float startLat, float startLong, float currentLat, float currentLong, float &North, float &East, float &Down)
{
  float heading = courseToWaypoint(startLat, startLong, currentLat, currentLong);
  float distance = distanceToWaypoint(startLat, startLong, currentLat, currentLong);
  North = distance * sin(heading);
  East = distance * cos(heading);
  // Down = currentRangeSensorHeight - heightOffset;
}


//// Generate minimum snap trajectory
// produces an array of nx9 states to track minimizing the snap of the linear movements
// TODO: the trajectory generator really doesn't like negative values for waypoints... can I fix this?
void NAVIGATION::minimum_snap_trajectory()
{
    // References:
    // Minimum Snap Trajectory Generation and Control for Quadrotors
    //

    Eigen::Matrix<double, 1, 8> temp_;          // store desired position, velocity, and acceleration calc before assignment

    if(_is_optimized == 0)
    {
        // Waypoints set in initialize_variables()
        generate_ts();          // stores the time-series goals in _ts
        min_snap_optimization();
        _is_optimized = 1;

    } else if(_start_traj == 0) {
        ;                               // wait until trajectory call begins
        _traj_start_time = sim_time;
    } else if(_start_traj == 1)
    {
        _traj_time = sim_time - _traj_start_time;   // track trajectory specific time

        int idx_ = 0;                    // _ts index of current operation

        // search the trajectory time-series to find the current index
        for(int i = 0; i < _ts.rows(); i++)
        {
            if(_traj_time < _ts(i)){
                idx_ = i;
                break;
            } else if ((_ts.rows()-1) == i) {
                idx_ = i;       // capture final case
            }
        } // end time series search

        // seems to work better when I do this; captures the 0 case
        if(idx_ == 0){
            idx_ = 0;
        } else {
            idx_ = idx_ - 1;
        } // end index correction and error catching

        // check if this is the last index and if the quadcopter is there; then finish
        if(((idx_+2) == _ts.rows()) & ((_traj_setpoints.block(idx_+1, 0, 1, 3) - _sensor_pos).lpNorm<2>() < 0.2))
        {
            std::cout );
            Serial.println("*** Trajectory finished *** " );
            Serial.println("Desired finish time: " << _ts(idx_+1) );
            Serial.println("Actual finish time: " << _traj_time );
            _traj_finished = 1;
        }

        // Apply optimized coefficients to the setpoints
        // Generate desired position
        temp_ << std::pow(_traj_time, 7), std::pow(_traj_time, 6), std::pow(_traj_time, 5), std::pow(_traj_time, 4),
                    std::pow(_traj_time, 3), std::pow(_traj_time, 2), _traj_time, 1.0;
        _desired_pos = temp_ * _coef.block(8*idx_, 0, 8, 3);

        // Generate desired velocity
        temp_ << 7.0 * std::pow(_traj_time, 6), 6.0 * std::pow(_traj_time, 5), 5.0 * std::pow(_traj_time, 4),
                    4.0 * std::pow(_traj_time, 3), 3.0 * std::pow(_traj_time, 2), 2.0 * _traj_time, 1.0, 0.0;
        _desired_vel = temp_ * _coef.block(8*idx_, 0, 8, 3);

        // Generate desired acceleration
        temp_ << 42.0 * std::pow(_traj_time, 5), 30.0 * std::pow(_traj_time, 4), 20.0 * std::pow(_traj_time, 3),
                    12.0 * std::pow(_traj_time, 2), 6.0 * _traj_time, 2.0, 0.0, 0.0;
        _desired_acc = temp_ * _coef.block(8*idx_, 0, 8, 3);

    } // end if
} // end minimum_snap_trajectory()

//// Generate time-series for the minimum snap trajectory
// produces a nx1 array of times which the vehicle needs to waypoint
void void NAVIGATION::generate_ts()
{
    double speed_ = 1.75;         // m/s                    // 1.75 m/s is pretty much the limit with hover-envelope, final error increased to 0.2m
    int _path_max_size = _traj_setpoints.rows();
    double path_len_;
    path_len_ = (_traj_setpoints.middleRows(1, _path_max_size-1)
            - _traj_setpoints.middleRows(0, _path_max_size-1)).array().pow(2).rowwise().sum().cwiseSqrt().sum();
    _total_traj_time = path_len_/speed_;         // based on naive path length
    Eigen::MatrixXd path_seg_lenth_((_path_max_size-1),1);
    path_seg_lenth_ = (_traj_setpoints.middleRows(1, _path_max_size-1)
                         - _traj_setpoints.middleRows(0, _path_max_size-1)).array().pow(2).rowwise().sum().cwiseSqrt().cwiseSqrt();

    double cumsum_ = 0;
    for(int i=0; i<(_path_max_size-1); i++)
    {
        _ts(i, 0) = path_seg_lenth_(i) + cumsum_;
        cumsum_ += path_seg_lenth_(i,0);
    }

    double scaling_ = _ts(_path_max_size-2, 0);
    _ts = _ts.array() / scaling_;       // scaled/normalized time series

    Eigen::MatrixXd prev_ts_(_ts.rows(), _ts.cols());
    prev_ts_ = _ts;                     // store previous ts to add 0.0 at the start

    _ts.resize(_path_max_size, 1);
    _ts << 0.0, prev_ts_;               // add in 0.0 seconds for first time

    _ts = _ts * _total_traj_time;      // final goal times for each waypoint (to be shifted later)

} // end generate_ts()

//// Minimum snap trajectory optimization
// find the optimizing coefficients for the trajectory
void void NAVIGATION::min_snap_optimization()
{
    int m_ = _traj_setpoints.rows();            // wpts
    int n_ = _traj_setpoints.cols();            // x,y,z
    m_ = m_ - 1;                                // mathematical convenience         // TODO: I think this is an error; cuts off last trajectory

    //    double eps_ = 2e-52;                        // TODO: can I use float here? if not... then limits?
    double eps_ = 2e-10;
    _coef.resize(8*m_, 3);            // needs to match setpoints * constraints dimensions

    // define template matrices
    Eigen::MatrixXd X_(8*m_, n_);
    Eigen::MatrixXd Y_(8*m_, n_);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_;
    A_.resize(8*m_, 8*m_);

    // https://stackoverflow.com/questions/31159196/can-we-create-a-vector-of-eigen-matrix
    std::vector<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > A_vec_;

    // zero the matrices
    X_.setZero();
    Y_.setZero();
    A_.setZero();
    A_ = A_.setIdentity();
    A_ = A_ * eps_;                 // condition the matix so inversions aren't singular

//    m_ = m_ - 1;                                // mathematical convenience         // TODO: I think this is an error; cuts off last trajectory

    // initialize vector of A_ matrices
    for(int i = 0; i < n_; i++)
    {
        A_vec_.push_back(A_);
//        Serial.println(A_vec_[i] );
//        Serial.println(A_vec_[i](0,0) );
//        Serial.println(std::endl;
    }

    // In a 7th order minimum-snap trajectory there are 8 parameters for each subpath
    for(int i = 0; i < n_; i++)
    {
        int idx_ = 0;
        Eigen::Matrix<double, 1, 8> temp_;
//        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> temp_;
//        temp_.resize(1, 8);         // this size stays fairly consistent

        // Constraint 1: x_k(t_k) = x_{k+1}(t_k) = p_k, where p_k is a waypoint
        for(int k = 0; k < m_-1; k++)
        {
            // A[idx, 8*k:8*(k+1), i]
            temp_ << std::pow(_ts(k+1), 7), std::pow(_ts(k+1), 6), std::pow(_ts(k+1), 5), std::pow(_ts(k+1), 4),
                        std::pow(_ts(k+1), 3), std::pow(_ts(k+1), 2), _ts(k+1), 1.0;
            A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;

            // Y[idx, i] = path0[k+1, i]
            Y_(idx_, i) = _traj_setpoints((k+1), i);
            idx_ += 1;                                      // iterate

            // A[idx, 8*(k+1):8*(k+2), i]
            temp_ << std::pow(_ts(k+1), 7), std::pow(_ts(k+1), 6), std::pow(_ts(k+1), 5), std::pow(_ts(k+1), 4),
            std::pow(_ts(k+1), 3), std::pow(_ts(k+1), 2), _ts(k+1), 1.0;
            A_vec_[i].block(idx_, (8*(k+1)), 1, 8) = temp_;

            // Y[idx, i] = path0[k+1, i]
            Y_(idx_, i) = _traj_setpoints((k+1), i);
            idx_ += 1;                                      // iterate

        } // end contraint 1

        // Constraint 2: \dot{x}_k(t_k) = \dot{x}_{k+1}(t_k)
        for(int k = 0; k < m_-1; k++)
        {
            // A[idx, 8*k:8*(k+1), i]
            temp_ << 7.0*std::pow(_ts(k+1), 6), 6.0*std::pow(_ts(k+1), 5), 5.0*std::pow(_ts(k+1), 4),
                        4.0*std::pow(_ts(k+1), 3), 3.0*std::pow(_ts(k+1), 2), 2.0*_ts(k+1), 1.0, 0.0;
            A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;

            // A[idx, 8*(k+1):8*(k+2), i]
            temp_ << -7.0*std::pow(_ts(k+1), 6), -6.0*std::pow(_ts(k+1), 5), -5.0*std::pow(_ts(k+1), 4),
                        -4.0*std::pow(_ts(k+1), 3), -3.0*std::pow(_ts(k+1), 2), -2.0*_ts(k+1), -1.0, 0.0;
            A_vec_[i].block(idx_, (8*(k+1)), 1, 8) = temp_;
            Y_(idx_, i) = 0.0;
            idx_ += 1;                                      // iterate

        } // end constraint 2

        // Constraint 3: \ddot{x}_k(t_k) = \ddot{x}_{k+1}(t_k)
        for(int k = 0; k < m_-1; k++)
        {
            // A[idx, 8*k:8*(k+1), i]
            temp_ << 42.0*std::pow(_ts(k+1), 5), 30.0*std::pow(_ts(k+1), 4), 20.0*std::pow(_ts(k+1), 3),
                        12.0*std::pow(_ts(k+1), 2), 6.0*_ts(k+1), 2.0, 0.0, 0.0;
            A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;

            // A[idx, 8*(k+1):8*(k+2), i]
            temp_ << -42.0*std::pow(_ts(k+1), 5), -30.0*std::pow(_ts(k+1), 4), -20.0*std::pow(_ts(k+1), 3),
                        -12.0*std::pow(_ts(k+1), 2), -6.0*_ts(k+1), -2.0, 0.0, 0.0;
            A_vec_[i].block(idx_, (8*(k+1)), 1, 8) = temp_;
            Y_(idx_, i) = 0.0;
            idx_ += 1;                                      // iterate

        } // end constraint 3

        // Constraint 4: x^(3)_k(t_k) = x^(3)_{k+1}(t_k)
        for(int k = 0; k < m_-1; k++)
        {
            // A[idx, 8*k:8*(k+1), i]
            temp_ << 210.0*std::pow(_ts(k+1), 4), 120.0*std::pow(_ts(k+1), 3), 60.0*std::pow(_ts(k+1), 2),
                        24.0*_ts(k+1), 6.0, 0.0, 0.0, 0.0;
            A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;

            // A[idx, 8*(k+1):8*(k+2), i]
            temp_ << -210.0*std::pow(_ts(k+1), 4), -120.0*std::pow(_ts(k+1), 3), -60.0*std::pow(_ts(k+1), 2),
                    -24.0*_ts(k+1), -6.0, 0.0, 0.0, 0.0;
            A_vec_[i].block(idx_, (8*(k+1)), 1, 8) = temp_;
            Y_(idx_, i) = 0.0;
            idx_ += 1;                                      // iterate

        } // end constraint 4

        // Constraint 5: x^(4)_k(t_k) = x^(4)_{k+1}(t_k)
        for(int k = 0; k < m_-1; k++)
        {
            // A[idx, 8*k:8*(k+1), i]
            temp_ << 840.0*std::pow(_ts(k+1), 3), 360.0*std::pow(_ts(k+1), 2), 120.0*_ts(k+1), 24.0, 0.0, 0.0,
                        0.0, 0.0;
            A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;

            // A[idx, 8*(k+1):8*(k+2), i]
            temp_ << -840.0*std::pow(_ts(k+1), 3), -360.0*std::pow(_ts(k+1), 2), -120.0*_ts(k+1), -24.0, 0.0, 0.0,
                        0.0, 0.0;
            A_vec_[i].block(idx_, (8*(k+1)), 1, 8) = temp_;
            Y_(idx_, i) = 0.0;
            idx_ += 1;                                      // iterate

        } // end constraint 5

        // Constraint 6: x^(5)_k(t_k) = x^(5)_{k+1}(t_k)
        for(int k = 0; k < m_-1; k++)
        {
            // A[idx, 8*k:8*(k+1), i]
            temp_ << 2520.0*std::pow(_ts(k+1), 2), 720.0*_ts(k+1), 120.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;

            // A[idx, 8*(k+1):8*(k+2), i]
            temp_ << -2520.0*std::pow(_ts(k+1), 2), -720.0*_ts(k+1), -120.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            A_vec_[i].block(idx_, (8*(k+1)), 1, 8) = temp_;
            Y_(idx_, i) = 0.0;
            idx_ += 1;                                      // iterate

        } // end constraint 6

        // Constraint 7: x^(6)_k(t_k) = x^(6)_{k+1}(t_k)
        for(int k = 0; k < m_-1; k++)
        {
            // A[idx, 8*k:8*(k+1), i]
            temp_ << 5040.0*_ts(k+1), 720.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;

            // A[idx, 8*(k+1):8*(k+2), i]
            temp_ << -5040.0*_ts(k+1), -720.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            A_vec_[i].block(idx_, (8*(k+1)), 1, 8) = temp_;
            Y_(idx_, i) = 0.0;
            idx_ += 1;                                      // iterate

        } // end constraint 7

        // 4 of the last 8 constraints to be added: (already 8(m-1) in place...
        int k = 0;

        // from constraint 1
        temp_ << std::pow(_ts(k), 7), std::pow(_ts(k), 6), std::pow(_ts(k), 5), std::pow(_ts(k), 4),
                    std::pow(_ts(k), 3), std::pow(_ts(k), 2), _ts(k), 1.0;
        A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;
        Y_(idx_, i) = _traj_setpoints(k, i);
        idx_ += 1;                                      // iterate

        // from constraint 2
        temp_ << 7.0*std::pow(_ts(k), 6), 6.0*std::pow(_ts(k), 5), 5.0*std::pow(_ts(k), 4),
                    4.0*std::pow(_ts(k), 3), 3.0*std::pow(_ts(k), 2), 2.0*_ts(k), 1.0, 0.0;
        A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;
        Y_(idx_, i) = 0.0;
        idx_ += 1;                                      // iterate

        // from constraint 3
        temp_ << 42.0*std::pow(_ts(k), 5), 30.0*std::pow(_ts(k), 4), 20.0*std::pow(_ts(k), 3),
                    12.0*std::pow(_ts(k), 2), 6.0*_ts(k), 2.0, 0.0, 0.0;
        A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;
        Y_(idx_, i) = 0.0;
        idx_ += 1;                                      // iterate

        //from constraint 4
        temp_ << 210.0*std::pow(_ts(k), 4), 120.0*std::pow(_ts(k), 3), 60.0*std::pow(_ts(k), 2),
                24.0*_ts(k), 6.0, 0.0, 0.0, 0.0;
        A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;
        Y_(idx_, i) = 0.0;
        idx_ += 1;                                      // iterate

        // Last 4 of the last 8 constraints to be added:
        k = m_-1;
//        k = m_;
//        Serial.println("vec: " << i );
//        Serial.println(A_vec_[i] );
//        Serial.println(std::endl;
//        Serial.println(A_vec_[i].size() );
//        Serial.println(std::endl
//        Serial.println(_ts );
//        Serial.println(std::endl;

        // from constraint 1
        temp_ << std::pow(_ts(k+1), 7), std::pow(_ts(k+1), 6), std::pow(_ts(k+1), 5), std::pow(_ts(k+1), 4),
                    std::pow(_ts(k+1), 3), std::pow(_ts(k+1), 2), _ts(k+1), 1.0;
        A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;
        Y_(idx_, i) = _traj_setpoints(k+1, i);
        idx_ += 1;                                      // iterate

        // from constraint 2
        temp_ << 7.0*std::pow(_ts(k+1), 6), 6.0*std::pow(_ts(k+1), 5), 5.0*std::pow(_ts(k+1), 4),
                    4.0*std::pow(_ts(k+1), 3), 3.0*std::pow(_ts(k+1), 2), 2.0*_ts(k+1), 1.0, 0.0;
        A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;
        Y_(idx_, i) = 0.0;
        idx_ += 1;                                      // iterate

        // from constraint 3
        temp_ << 42.0*std::pow(_ts(k+1), 5), 30.0*std::pow(_ts(k+1), 4), 20.0*std::pow(_ts(k+1), 3),
                    12.0*std::pow(_ts(k+1), 2), 6.0*_ts(k+1), 2.0, 0.0, 0.0;
        A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;
        Y_(idx_, i) = 0.0;
        idx_ += 1;                                      // iterate

        //from constraint 4
        temp_ << 210.0*std::pow(_ts(k+1), 4), 120.0*std::pow(_ts(k+1), 3), 60.0*std::pow(_ts(k+1), 2),
                    24.0*_ts(k+1), 6.0, 0.0, 0.0, 0.0;
        A_vec_[i].block(idx_, (8*k), 1, 8) = temp_;
        Y_(idx_, i) = 0.0;
        idx_ += 1;                                      // iterate

        // solve linear system of equations
        // https://stackoverflow.com/questions/53247078/c-eigen-for-solving-linear-systems-fast
        // https://eigen.tuxfamily.org/dox/TopicUsingIntelMKL.html
        _coef.col(i) = A_vec_[i].lu().solve(Y_.col(i));

    } // end for(i)  -- position dimensions x, y, z
} // end min_snap_optimization()


//// Generates a Gerono Lemniscate trajectory for tracking
// centered on the origin at (0.0, 0.0)
void NAVIGATION::figure_eight_trajectory() {
    // References
    // Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of High-Speed Trajectories
    //
    // Also called: the Gerono Lemniscate trajectory
    //

    double pos_scaling_ = 2.5;          // larger scales the trajectory up in size
    double time_scale_ = 0.9;           // lower is slower (all the way down to 0.0)

    if (!_start_traj) {
        _desired_pos << 1.0, 1.0, _desired_pos(2);
    }

    if (!_start_traj & ((_desired_pos - _sensor_pos).lpNorm<2>() < 0.01)) {
        std::cout << "Starting Figure-8 Trajectory... " << std::endl;
        _start_traj = 1;
        _traj_start_time = sim_time;
    } else if (_start_traj) {
        _traj_time = sim_time - _traj_start_time;
        _desired_pos << pos_scaling_ * cos(time_scale_*_traj_time), pos_scaling_ * sin(time_scale_*_traj_time) * cos(time_scale_*_traj_time), _desired_pos(2);
        _desired_vel << -pos_scaling_*time_scale_ * sin(time_scale_*_traj_time),
                pos_scaling_*time_scale_ * (pow(cos(time_scale_*_traj_time), 2) - pow(sin(time_scale_*_traj_time), 2)), 0.0;
        _desired_acc << -pos_scaling_*time_scale_*time_scale_ * cos(time_scale_*_traj_time),
                            -pos_scaling_*time_scale_*time_scale_ * 4.0* sin(time_scale_*_traj_time)*cos(time_scale_*_traj_time), 0.0;
    }
} // end figure_eight_trajectory()


//// Basic continuous circling trajectory
// circles around the origin (0.0, 0.0) at a constant altitude
void NAVIGATION::circling_trajectory()
{
    // References:
    // https://gamedev.stackexchange.com/questions/9607/moving-an-object-in-a-circular-path#:~:text=You%20can%20do%20that%20using,radius%20is%20its%20radius.
    // https://math.stackexchange.com/questions/26329/formula-to-move-the-object-in-circular-path
    // Fast Nonlinear Model Predictive Control for Multicopter Attitude Tracking on SO(3)

    // TODO: have the yaw vector always pointing to the origin, maybe by transforming the euler att to world frame?
    // TODO: either fix the random spikes in roll/pitch/yaw derived measurements or filter them
    // TODO: figure out how to generate a constant desired velocity to be tracked with this trajectory method

    double radius_ = 2.0;          // circle radius
    if(!_start_traj) {
        _desired_pos << 1.0, 0.0, _desired_pos(2);
    }

    if(!_start_traj & ((_desired_pos - _sensor_pos).lpNorm<2>() < 0.01))
    {
        std::cout << "Starting Circling Trajectory... " << std::endl;
        _start_traj = 1;
        _traj_start_time = sim_time;
    } else if(_start_traj){
        _traj_time = sim_time - _traj_start_time;
        _desired_pos << radius_*cos(_traj_time), radius_*sin(_traj_time), _desired_pos(2);
        _desired_vel << -radius_*sin(_traj_time), radius_*cos(_traj_time), 0.0;
        _desired_acc << -radius_*cos(_traj_time), -radius_*sin(_traj_time), 0.0;
    }
}


//// Hover-envelope position controller

void NAVIGATION::basic_position_controller()
{
    // References (papers):
    // Trajectory Generation and Control for Precise Aggressive Maneuvers with Quadrotors
    // The GRASP Multiple Micro UAV Testbed
    //

    // produce desired acceleration vector; includes feed-forward acceleration term
    Eigen::Array3d acc_des_ = _desired_acc + (1.0*_Kd_pos.cwiseProduct(_desired_vel - _derived_lin_vel))
                                + (1.0*_Kp_pos.cwiseProduct(_desired_pos - _sensor_pos));

    // calculate desired euler attitudes for the attitude controller
    _desired_euler_att(0) = (1.0/_gravity) * ((acc_des_(0)*sin(_desired_euler_att(2))) + (acc_des_(1)*cos(_desired_euler_att(2))));
    _desired_euler_att(1) = (1.0/_gravity) * ((acc_des_(0)*-1.0*cos(_desired_euler_att(2))) + (acc_des_(1)*sin(_desired_euler_att(2))));
    _desired_euler_att(2) = _derived_euler_att(2);                    // desired yaw is forward-facing
//    _desired_euler_att(2) = _orig_desired_euler_att(2);     // yaw is a free variable

    // For testing the attitude controller; circumvents the position controller except for hover/altitude controller
//    _desired_euler_att(0) = _orig_desired_euler_att(0);
//    _desired_euler_att(1) = _orig_desired_euler_att(1);
//    _desired_euler_att(2) = _orig_desired_euler_att(2);

    _desired_tot_thrust_delta = (_mass / (8.0 * _motor_force_const * _hover_point)) * acc_des_(2);      // hover/altitude control

} // end basic_position_controller()