// depth_filter.cpp --smoothes the /depth signal

// What this code does
// -------------------
// * SUBSCRIBES to /depth (std_msgs/Float64), which the Python
// depth_publisher node publishes

// * Each time a reading arrives, runs it through a first order low-pass filter(an exponentional moving average)
//     y[k] = alpha * x[k] + (1-alpha) * y[k-1]
// * PUBLISHES the smoothed values on depth_filtered

// It is a "filter in the pipe": data in -> processing -> dataout.
// It does not know or care that its input comes from a python node

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

// Every c++ node is a class that inherits from rclcpp::Node
class DepthFilter : public rclcpp::Node
{
public:
// Constructor: runs once when the node is created. Like python's __init__,
// it only SETS UP the publisher, subscription and parameter. No data from flows yet

    DepthFilter() : Node("depth_filter") //call the parent constructor
    {
        // PARAMETER: a named setting you can change without recompiling,
        // eg ros2 run sub_cpp depth_filter --ros-args -p alpha:=0.5
        // declare_parameter<double>(name, default) registers it with ROS and
        // returns the value given on the command line, or the default (0.2)
        
        alpha_ = declare_parameter<double>("alpha", 0.2);

        // PUBLISHER for the output. The <...> template argument is the message type.
        // Topic depth_filtered (-> /depth_filtered), que depth 10
        pub_ = create_publisher<std_msgs::msg::Float64>("depth_filtered", 10);

        // SUBSCRIPTION to the input. Arguments:
        //     "depth" -> topic to listen to (/depth)
        //     10      -> que depth
        //     lamda   -> the callback: code ROS runs for every message on /depth

        //     A lambda is an unnamed function written inline. [this] "captures" the
        //     current object so the lambda can use alpha_, y_, and pub_. The parameter
        //     'const std_msgs::msg::Float64 & msg' is a read only reference to the
        //     incoming message (no copy is made)
        sub_ = create_subscription<std_msgs::msg::Float64>("depth", 10,
            [this](const std_msgs::msg::Float64 & msg) {
                // First sample: start the filter at the measured value instead of 0,
                // so the output does not ramp up from zero.
                if (!initialized_){
                    y_ = msg.data;
                    initialized_ = true;
                }
            
            // The filter itself: blend the new reading with the previouys output.
            y_ = alpha_ * msg.data + (1.0 - alpha_) * y_;

            // Build the ouput message and publish it.
            std_msgs::msg::Float64 out; // C++ messages are plain structs
            out.data = y_;              // same field names as in python
            pub_->publish(out);         // '->' because pub_ is a (smart) pointer
            } // end of callback lambda
        );    // end of create_subscription(...)
    } // end of constructor DepthFilter

private:
    // Member variables. The trailing underscore is ROS C++ style convention
    // meaning "this belongs to the class", so it is easy to tell from local variables

    double alpha_;  // filter coefficient from the parameter
    double y_{0.0}; // filter state: the previous output
    bool initialized_{false}; // has the first sample arrived ter?

    // Handles the publisher and subscription. They MUST be stored as members,
    // A SharedPtr deletes its object when the last copy goes away; if these were
    // local variables inthe constructor, the subscription would be destroyed at the 
    // end of the constructor and the node would silently recieve nothing.

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;

}; // end of class DepthFilter (a class definition must end with a semicolon)

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv); //step 1: start the C++ client library (always first)

    //steps 2 and 3: build the node on the heap (make_shared) and spin it
    // spin() sleeps until a message arrives on /depth, runs the lamda above,
    // and repeats until Ctrl+C.
    rclcpp::spin(std::make_shared<DepthFilter>());

    rclcpp::shutdown(); // step 5: cleanup
    return 0;
} //end of main