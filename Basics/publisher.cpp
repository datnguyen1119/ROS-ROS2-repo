#include <chrono>   //for time-related utilities (used for timer duration)
#include <functional>   //for std::bind, which is used to bind member functions. (!) what is bind member function?
#include <memory>   //for smart pointers like std::shared_ptr.
#include <string>   // for std::string

#include "rclcpp/rclcpp.hpp"    //Core ROS 2 C++ client library.
#include "std_msgs/msg/string.hpp"  //message type used for publishing.

using namespace std::chrono_literals; //Allows using 500ms instead of writing std::chrono::milliseconds(500).
// these libs above, i have examples for them in libraries folder


class MinimalPublisher : public rclcpp::Node 
/*
You're defining a custom node named MinimalPublisher.
public rclcpp::Node means this class inherits from ROS 2’s core Node class.
this is inheritance in OOP
*/
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    /*
    This is the constructor. It initializes the object when it's created.

    Contructor is a special function that runs automatically when an object of a class is created
    variables are initialized, meaning they are given allocated memory and value, if not when the variable are used, its value will be a random number

    Node("minimal_publisher"): Calls the base class constructor (from rclcpp::Node) and assigns the name "minimal_publisher" to the node.
    count_(0): Initializes the counter to 0.
    */
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      /*
      create_publisher is a member function of Node (inherited).
      It sets up a publisher of type String on topic "topic" with a queue size of 10.
      publisher_ is a private member, encapsulated within the class (OOP: encapsulation).
      */
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
      /*
      create_wall_timer() sets up a repeating timer.
      Every 500ms, it calls the method timer_callback().
      the std::bind - Binds the non-static member function timer_callback to the current instance (this), making it callable like a function.
      */
    }

  private:
    void timer_callback()
    //This is the member function called every 500ms. It’s private, so it’s encapsulated and only used internally.
    {
      auto message = std_msgs::msg::String(); //auto type?
      /*
      message is defined with auto type?
      */
      message.data = "Hello, world! " + std::to_string(count_++);
      /*
      data member of class message is from std_msgs
      data: 
        Hello, worlds! 1
        Hello, worlds! 2
        Hello, worlds! 3
      */
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      /*
      This is a macro provided by ROS 2 (rclcpp) for logging messages at the INFO level.
      It prints formatted messages to the terminal (or log file) only if the log level is INFO or more verbose (DEBUG, TRACE).
      the message output will be in this form:
      [INFO] [minimal_publisher]: Publishing: 'Hello, world! 5'

      this may helps with seeing what node is doing in real time, which helps monitor know where the problem is

      */
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    /*
    publisher_ is a private member variable of the MinimalPublisher class.
    The trailing underscore _ is a naming convention that signals this is a class member variable
    this is often encounter in OOP design

    and the define variables in private part is for OOP design to, the aim is to expose what is important first, then for more details
    of what happen dev can scroll down to read
    */
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //Initializes the ROS 2 system (sets up internals, communication, etc.).

  rclcpp::spin(std::make_shared<MinimalPublisher>());
  /*
  spin() - Keeps the node alive, handling incoming events (like timers, subscriptions).
  A std::shared_ptr<T> is a smart pointer provided by C++ that:
  - Automatically manages memory.
  - Keeps a reference count of how many things are using the object.
  - When the last reference goes away, it automatically deletes the object.
  - Eliminates manual delete calls → prevents memory leaks and dangling pointers.
  */

  rclcpp::shutdown();
  return 0;
  /*
  shutdown ROS2 and ends the program cleanly
  */
}