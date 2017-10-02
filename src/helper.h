#ifndef PLAN_UTIL
#define PLAN_UTIL

#include <math.h>
#include <cmath>
#include <vector>
#include <string>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <map>
#include <tuple>
#include <iostream>

using namespace std;

// unnamed namespace only because the implementation is in this
// header file and we don't want to export symbols to the obj files
namespace
{

namespace utils
{
    const double SAFE_FRONT_GAP = 22.0;
    const double SAFE_REAR_GAP = -15;
    const double LANE_WIDTH = 4.0;
    const int AVAILABLE_LANES = 3;
    const double MAX_S = 6914.14925765991;

    struct Car {
        Car()
        : x(0), y(0), s(0), d(0), yaw(0), v(0)
        {}
    
        Car(double _x, double _y, double _s, double _d, double _yaw, double v)
        : x(_x), y(_y), s(_s), d(_d), yaw(_yaw), v(v)
        {}
        
        double x, y, s, d, yaw, v;
    };

    struct Params {
        int lane = 1;
        double ref_vel = 0.0; //mph
        double speed_limit = 49.5; //mph
        double horizon = 50; //waypoints
        double horizon_dist = 30; //meters
        double dt = 0.02; //waypoint spacings in seconds
        double vel_step = 0.224; 
    } params;

    
    constexpr double pi() { return M_PI; }
    
    double deg2rad(double x) { return x * pi() / 180; }
    
    double rad2deg(double x) { return x * 180 / pi(); }
    
    double distance(double x1, double y1, double x2, double y2)
    {
        return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }
    
    int d2lane(double d){
        return (int)floor(d/LANE_WIDTH);
    }
    
    double lane2d(int lane){
        return LANE_WIDTH * (0.5 + lane);
    }
    
    int closestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
    {
    
        double closestLen = 100000; //large number
        int closestPoint = 0;
    
        for (int i = 0; i < maps_x.size(); i++)
        {
            double map_x = maps_x[i];
            double map_y = maps_y[i];
            double dist = distance(x, y, map_x, map_y);
            if (dist < closestLen)
            {
                closestLen = dist;
                closestPoint = i;
            }
        }
    
        return closestPoint;
    }
    
    int nextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
    {
    
        int closestPoint = closestWaypoint(x, y, maps_x, maps_y);
    
        double map_x = maps_x[closestPoint];
        double map_y = maps_y[closestPoint];
    
        double heading = atan2((map_y - y), (map_x - x));
    
        double angle = abs(theta - heading);
    
        if (angle > pi() / 4)
        {
            closestPoint++;
        }
    
        return closestPoint;
    }
    
    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
    {
        int next_wp = nextWaypoint(x, y, theta, maps_x, maps_y);
    
        int prev_wp;
        prev_wp = next_wp - 1;
        if (next_wp == 0)
        {
            prev_wp = maps_x.size() - 1;
        }
    
        double n_x = maps_x[next_wp] - maps_x[prev_wp];
        double n_y = maps_y[next_wp] - maps_y[prev_wp];
        double x_x = x - maps_x[prev_wp];
        double x_y = y - maps_y[prev_wp];
    
        // find the projection of x onto n
        double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
        double proj_x = proj_norm * n_x;
        double proj_y = proj_norm * n_y;
    
        double frenet_d = distance(x_x, x_y, proj_x, proj_y);
    
        //see if d value is positive or negative by comparing it to a center point
    
        double center_x = 1000 - maps_x[prev_wp];
        double center_y = 2000 - maps_y[prev_wp];
        double centerToPos = distance(center_x, center_y, x_x, x_y);
        double centerToRef = distance(center_x, center_y, proj_x, proj_y);
    
        if (centerToPos <= centerToRef)
        {
            frenet_d *= -1;
        }
    
        // calculate s value
        double frenet_s = 0;
        for (int i = 0; i < prev_wp; i++)
        {
            frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
        }
    
        frenet_s += distance(0, 0, proj_x, proj_y);
    
        return {frenet_s, frenet_d};
    }
    
    // Transform from Frenet s,d coordinates to Cartesian x,y
    vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
    {
        int prev_wp = -1;
    
        while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
        {
            prev_wp++;
        }
    
        int wp2 = (prev_wp + 1) % maps_x.size();
    
        double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
        // the x,y,s along the segment
        double seg_s = (s - maps_s[prev_wp]);
    
        double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
        double seg_y = maps_y[prev_wp] + seg_s * sin(heading);
    
        double perp_heading = heading - pi() / 2;
    
        double x = seg_x + d * cos(perp_heading);
        double y = seg_y + d * sin(perp_heading);
    
        return {x, y};
    }

    bool tooClose(double gap) {
        return (gap > SAFE_REAR_GAP && gap < SAFE_FRONT_GAP);
    }
    
    std::pair<int, double> optimalLane(int time_ticks, int target_s, Car car, vector<vector<double>> sensor_fusion)
    {
        auto gap_cost = [](int lane, map<int, double> &gaps) {
            return !tooClose(gaps[lane]) ? 0.0: 1E12;
        };
    
    
        map<int, double> gaps;
        for (int lane = 0; lane < AVAILABLE_LANES; lane++){
                gaps[lane] = MAX_S;
        }
    
        for (auto const &sf_car : sensor_fusion){
            
            int id = sf_car[0];
            double d = sf_car[6];
            //make sure that we dont pick up vehicle outside our side of the highway
            //if (d < (2 + 4 * car.lane + 2) && d > (2 + 4 * car.lane - 2))
            {
                int lane =  d2lane(d);
                double vx = sf_car[3];
                double vy = sf_car[4];
                double v = sqrt(vx * vx + vy * vy);
                double sf_s = sf_car[5];

                sf_s += ((double)time_ticks * .02 * v);

                double gap = sf_s - target_s;
                if (gap < (50 - MAX_S)) { // S looped around
                    gap += MAX_S;
                }

                if ( abs(gap) < abs(gaps[lane])){
                    gaps[lane] = gap;
                }
            }
        }
            
        int l = d2lane(car.d);
        double max_cost = gap_cost(l, gaps);
        int optimal_lane = l; //crude approach to prioritise staying in lane  - should be another cost function
        for (int i=max(0,l-1);i<min(AVAILABLE_LANES,l+2);i++){
            double cost = gap_cost(i, gaps);
            //cout << "lane: " << i << " gap: " << gaps[i] << " costs: " << cost << endl;
            if (cost < max_cost){
                optimal_lane = i;
                max_cost = cost;
            }
        }
        //cout << "current lane: " << l << endl;
        std::pair<int, double> result(optimal_lane, gaps[optimal_lane]);
        //cout << "Optimal lane: " << result.first << " gap: " << result.second << endl;
        return result;
    }
}
}
#endif //PLAN_UTIL