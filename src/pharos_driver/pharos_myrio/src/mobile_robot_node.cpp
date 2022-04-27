// ROS
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <algorithm>
#include <functional>
#include <cstdlib>

#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Vector3Stamped.h>
#include <pharos_myrio/MotorVelStamped.h>

using boost::asio::ip::udp;

class MyRioUDPNode
{
    enum
    {
        max_length = 1024
    };
    char data_[max_length];

public:
    // UDP
    boost::asio::io_service io_service_;
    udp::endpoint send_ep_;
    udp::endpoint receive_ep_;
    udp::socket socket_;
    boost::asio::deadline_timer deadline_;
    boost::posix_time::seconds timeout_;
    boost::system::error_code error_;
    int32_t publish_rate_;
    int32_t socket_timeout_;
    // UDP


    ros::NodeHandlePtr node_;
    ros::NodeHandlePtr pnode_;

    ros::Publisher pub_;
    ros::Subscriber sub_;

    std::string frame_id_;


    // CompactRIO IP address and port
    std::string myRIO_IP_;
    std::string myRIO_TX_Port_;
    std::string myRIO_RX_Port_;

    // geometry_msgs::Pose pose;
    pharos_myrio::MotorVelStamped rx_msg_;
    pharos_myrio::MotorVelStamped rx_msg_old_;


    boost::mutex mutex_;


    MyRioUDPNode() :
            socket_timeout_(10), socket_(io_service_), deadline_(io_service_),
            timeout_(socket_timeout_)
    {
    }

    int init()
    {
      // Use global namespace for node
      node_ = ros::NodeHandlePtr(new ros::NodeHandle());

      // Use private namespace for parameters
      pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));
      pnode_->param("publish_rate", publish_rate_, 100);

      pnode_->param("frame_id", frame_id_, std::string("myrio"));

      pnode_->param("udp/ip", myRIO_IP_, std::string("192.168.3.25"));
      pnode_->param("udp/tx_port", myRIO_TX_Port_, std::string("39000"));
      pnode_->param("udp/rx_port", myRIO_RX_Port_, std::string("39001"));
      pnode_->param("udp/socket_timeout", socket_timeout_, 10);

      // ROS_WARN_STREAM(pnode_->getNamespace() << "msg");
      // ROS_FATAL_STREAM(pnode_->getNamespace() << "msg");

      pub_ = node_->advertise<pharos_myrio::MotorVelStamped>("motor_vel", 10);
      sub_ = node_->subscribe<pharos_myrio::MotorVelStamped>("motor_cmd", 10, &MyRioUDPNode::MotionCommand, this);



      timeout_ = boost::posix_time::seconds(socket_timeout_);
      send_ep_ = udp::endpoint(boost::asio::ip::address::from_string(myRIO_IP_), std::atoi(myRIO_TX_Port_.c_str()));
      receive_ep_ = udp::endpoint(udp::v4(), std::atoi(myRIO_RX_Port_.c_str()));

      socket_.open(udp::v4());

      deadline_.expires_at(boost::posix_time::pos_infin);
      this->deadlineCallback(deadline_, socket_);

      return 0;
    }

    void deadlineCallback(boost::asio::deadline_timer& t, udp::socket& s)
    {
      if (t.expires_at() <= boost::asio::deadline_timer::traits_type::now())
      {
        s.cancel();
        t.expires_at(boost::posix_time::pos_infin);
      }
      t.async_wait(boost::bind(&MyRioUDPNode::deadlineCallback, this, boost::ref(t), boost::ref(s)));
    }

    void handleRead(const boost::system::error_code& ec, std::size_t ln)
    {

      error_ = ec;
      if (!socket_.is_open())
        return;

      if (!ec)
      {
        std::string msg(data_);
        std::istringstream dataStream;


        if (!msg.empty())
        {
          boost::mutex::scoped_lock lock(mutex_);

          rx_msg_.header.stamp = ros::Time::now();
          rx_msg_.header.frame_id = frame_id_;


          // parser
          dataStream.str(msg);
          dataStream
          >> rx_msg_.state
          >> rx_msg_.vel.left
          >> rx_msg_.vel.right;
        }
        else
        {
          ROS_WARN_STREAM("Empty message");
        }
      }
    }

    void MotionCommand(const pharos_myrio::MotorVelStampedConstPtr& cmd)
    {
      std::ostringstream ss;

      ss.unsetf(std::ios::floatfield);
      // set %7.7f
      ss.precision(7);
      ss.width(7);

      ss << cmd->header.seq
      << " " << cmd->vel.left
      << " " << cmd->vel.right
      << " " << cmd->state
      << "\n";

      try
      {
        socket_.send_to(boost::asio::buffer(std::string(ss.str())), send_ep_);
      }
      catch (std::exception& e)
      {
        ROS_ERROR_STREAM(e.what());
      }

    }


    void ReadMyRIO()
    {
      udp::socket socket2(io_service_, receive_ep_);
      while (node_->ok())
      {
        try
        {
          deadline_.expires_from_now(timeout_);
          error_ = boost::asio::error::would_block;

          socket2.async_receive_from(boost::asio::buffer(data_, max_length), receive_ep_,
                                     boost::bind(&MyRioUDPNode::handleRead, this, _1, _2));

          do
          {
            io_service_.run_one();
          } while (error_ == boost::asio::error::would_block);

          if (error_)
          {
            if (error_ == boost::asio::error::operation_aborted)
            {
              ROS_WARN("Socket receive timed out");
            }
            else
            {
              throw boost::system::system_error(error_);
            }
          }
        }
        catch (std::exception& e)
        {
          ROS_ERROR_STREAM(e.what());
        }
      }
    }

    void UDP_loop()
    {
      ros::Rate loop_rate(publish_rate_);
      ros::AsyncSpinner spinner(2);
      // handle communication with myrio in separate thread
      boost::thread myrio_read_thread(&MyRioUDPNode::ReadMyRIO, this);

      spinner.start();

      while (node_->ok())
      {
        // lock variables from being modified
        // by myrio communication thread
        {
          boost::mutex::scoped_lock lock(mutex_);

          if(rx_msg_.header.stamp == rx_msg_old_.header.stamp) rx_msg_.state = 0;
          else rx_msg_.state = 1;
          pub_.publish(rx_msg_);
          rx_msg_old_ = rx_msg_;
        }
        //    ros::spinOnce();
        loop_rate.sleep();
      }
      myrio_read_thread.join();
    }
};

int main(int argc, char** argv)
{
  //-- Init ROS node
  ros::init(argc, argv, "myrio_udp");
  MyRioUDPNode myRIO_ROS;

  if(!myRIO_ROS.init()) myRIO_ROS.UDP_loop();
  else return -1;

  return 0;
}
