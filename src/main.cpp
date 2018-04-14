#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>


#define track_length 1250

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID steer_pid;
  PID speed_pid;
  int n = 0;

  // Speed controller
  speed_pid.Init(0.06, 0.0002, 1, 500, 500);

  // Twiddle parameters
  double p[] = {0.16, 0.004, 3};
  double dp[] = {0.04, 0.001, 0.7};
  bool try_opposite[] = {true, true, true};
  int dp_index = 0;
  double best_error = std::numeric_limits<double>::max();

  steer_pid.Init(p[0], p[1], p[2], track_length - 200);
  printf("\nInitialization: PID: %5.3f %5.3f %5.3f  DP: %5.3f %5.3f %5.3f \n\n",
         p[0], p[1], p[2], dp[0], dp[1], dp[2]);

  h.onMessage([&steer_pid, &speed_pid, &n, &p, &dp, &best_error, &dp_index, &try_opposite](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;

          steer_pid.UpdateError(cte);
          steer_value = steer_pid.GetOutput();

          double speed_error = speed - 40;  // Target speed 60 mph
          speed_pid.UpdateError(speed_error);
          double throttle = speed_pid.GetOutput();

          n++;

          if (n%100 == 0) {
            printf("n=%d cte=%6.3f steer=%6.3f speed_error=%6.3f throttle=%6.3f  total error=%7f\n", n, cte, steer_value, speed_error,
                   throttle, steer_pid.TotalError());
          }

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);


          if (n % track_length  == 0)
          {
            if (n == track_length )
            {
              // First iteration
              best_error = steer_pid.TotalError();
              p[dp_index] += dp[dp_index];
              printf("\nInitialization: best error = %7f\n", best_error);
              printf("PID: %5.3f %5.3f %5.3f  DP: %5.3f %5.3f %5.3f \n\n", p[0], p[1], p[2], dp[0], dp[1], dp[2]);
              steer_pid.Init(p[0], p[1], p[2], track_length-200);
            }
            else {

              bool optimize_next_parameter = false;

              // optimize PID parameters
              if (steer_pid.TotalError() <= best_error) {
                best_error = steer_pid.TotalError();
                dp[dp_index] *= 1.1;
                optimize_next_parameter = true;
                try_opposite[dp_index] = true;
              } else {
                if (try_opposite[dp_index]) {
                  p[dp_index] -= 2 * dp[dp_index];
                } else {
                  p[dp_index] += dp[dp_index];
                  dp[dp_index] *= 0.9;
                  optimize_next_parameter = true;
                }
                try_opposite[dp_index] = !try_opposite[dp_index];
              }

              if(optimize_next_parameter) {
                // switch to next parameter
                dp_index = (dp_index + 1) % 3;
                printf("New parameter index: %d", dp_index);
                p[dp_index] += dp[dp_index];
              }

              printf("\nIteration %d  best error = %7f  current error = %7f   PID: %5.3f %5.3f %5.3f  DP: %5.3f %5.3f %5.3f \n\n",
                     n / track_length , best_error, steer_pid.TotalError(), p[0], p[1], p[2], dp[0], dp[1], dp[2]
              );
              steer_pid.Init(p[0], p[1], p[2], track_length-200);
            }
          }
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
