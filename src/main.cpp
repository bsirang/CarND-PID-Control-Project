#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>

#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// Calculate a steering angle that would achieve a desired yaw rate based on
// a given forward velocity
static double desired_steer_angle_for_angular_velocity(double angular_velocity, double forward_velocity) {

  // Defining some vehicle parameters to calculate a reasonable turning radius...
  // These were chosen arbitrarily but within reason for a real vehicle
  static constexpr double a = 2.0; // wheel width
  static constexpr double c = 1.7; // axel width
  static constexpr double b = 4.0; // axel to axel length
  static constexpr double ac2 = (a - c) / 2.0;

  // This max turning angle was taken from the simulator
  static constexpr double max_steering_angle = 25.0 * pi() / 180.0;

  // how much time to rotate a full revolution at the given angular velocity
  double rotation_time = 2.0 * pi() / ::fabs(angular_velocity);

  // what's that distance for a given forward velocity
  double circumference = forward_velocity * rotation_time;

  // Our desired turning radius...
  double radius = circumference / pi();

  // Calculate the steering angle that would yield this turning radius
  double angle = ::asin(b / (radius - ac2));

  // Scale down to -1.0 to 1.0
  angle /= max_steering_angle;
  if (angular_velocity < 0.0) {
    angle = -angle;
  }

  return std::min(1.0, std::max(-1.0, angle));
}

int main() {
  uWS::Hub h;

  const double Kp = 0.25;
  const double Ki = 0.0;
  const double Kd = 1.3;
  const double KiLim = 0.2;
  const double KLpfAlpha = 0.8;

  PID pid(Kp, Ki, Kd, KiLim, KLpfAlpha);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          // double angle = std::stod(j[1]["steering_angle"].get<string>());
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
           double desired_angular_velocity = std::min(1.0, std::max(-1.0, pid.run(-cte)));
           double speed_ms = speed * 0.44704;
           double steer_value = desired_steer_angle_for_angular_velocity(desired_angular_velocity, speed_ms);

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Speed: " << speed_ms << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
