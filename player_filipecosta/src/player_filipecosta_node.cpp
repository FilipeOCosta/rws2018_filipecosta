#include <iostream>
#include <vector>

// Boost includes
#include <boost/shared_ptr.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <rws2018_libs/team.h>
#include <tf/transform_broadcaster.h>
#include <sstream>

#include <rws2018_msgs/MakeAPlay.h>

using namespace std;

namespace rws_filipecosta
{
class Player
{
public:
  // Constructor with the same name as the class
  Player(string argin_name)
  {
    name = argin_name;
  }

  int setTeamName(int index = 0 /*default value*/)
  {
    switch (index)
    {
      case 0:
        return setTeamName("red");
        break;
      case 1:
        return setTeamName("green");
        break;
      case 2:
        return setTeamName("blue");
        break;
      default:
        // cout << "wrong team index given. Cannot set team" << endl;
        ROS_WARN("wrong team index given. Cannot set team");
        break;
    }
  }

  // Set team name, if given a correct team name (accessor)
  int setTeamName(string argin_team)
  {
    if (argin_team == "red" || argin_team == "green" || argin_team == "blue")
    {
      team_name = argin_team;
      return 1;
    }
    else
    {
      // cout << "cannot set team name to " << argin_team << endl;
      ROS_WARN("cannot set team name to ");
      return 0;
    }
  }

  // Gets team name (accessor)
  string getTeamName(void)
  {
    return team_name;
  }

  string name;  // A public atribute

private:
  string team_name;
};

class MyPlayer : public Player
{
public:
  boost::shared_ptr<Team> red_team;
  boost::shared_ptr<Team> green_team;
  boost::shared_ptr<Team> blue_team;

  boost::shared_ptr<Team> my_team;
  boost::shared_ptr<Team> my_preys;
  boost::shared_ptr<Team> my_hunters;

  tf::TransformBroadcaster br;  // declare the broadcaster

  MyPlayer(string argin_name, string argin_team) : Player(argin_name)
  {
    red_team = boost::shared_ptr<Team>(new Team("red"));
    green_team = boost::shared_ptr<Team>(new Team("green"));
    blue_team = boost::shared_ptr<Team>(new Team("blue"));

    if (red_team->playerBelongsToTeam(name))
    {
      my_team = red_team;
      my_preys = green_team;
      my_hunters = blue_team;
      setTeamName("red");
    }
    else if (green_team->playerBelongsToTeam(name))
    {
      my_team = green_team;
      my_preys = blue_team;
      my_hunters = red_team;
      setTeamName("green");
    }
    else if (blue_team->playerBelongsToTeam(name))
    {
      my_team = blue_team;
      my_preys = red_team;
      my_hunters = green_team;
      setTeamName("blue");
    }

    printReport();
  }

  void move()
  {
    tf::Transform transform;  // declare the transformation object
    transform.setOrigin(tf::Vector3(-8, -8, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, M_PI / 4);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "filipecosta"));
  }

  void printReport()
  {
    // cout << "My name is " << name << "and my team is " << getTeamName() << endl;

    ROS_INFO("My name is %s and my team is %s", name.c_str(), (getTeamName().c_str()));
    // ROS_WARN("My name is %s and my team is %s", name.c_str(), (getTeamName().c_str()));
    // ROS_ERROR("My name is %s and my team is %s", name.c_str(), (getTeamName().c_str()));
  }
};
}  // end of namespace

int main(int argc, char **argv)
{
  // std::string player_name = "filipecosta";
  // Creating an instance of class Player
  // Player player(player_name);

  // std::cout << "Created an instance of class player with public name " << player.name << std::endl;
  // player.setTeamName();
  // std::cout << "team name " << player.getTeamName() << std::endl;

  ros::init(argc, argv, "filipecosta");

  ros::NodeHandle n;

  rws_filipecosta::MyPlayer my_player("filipecosta", "green");

  if (my_player.red_team->playerBelongsToTeam("filipecosta"))
  {
    // cout << "o filipe esta na equipa certa" << endl;
    ROS_INFO("o filipe esta na equipa certa");
  };

  // ros::NodeHandle node;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    my_player.move();

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();
}
