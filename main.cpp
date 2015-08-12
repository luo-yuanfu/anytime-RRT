/*
 * anytimeRRT.cpp
 *
 *  Created on: Nov 8, 2014
 *      Author: Luo Yuanfu
 *  This Project is based on the paper:
	Ferguson, Dave, and Anthony Stentz. "Anytime rrts."
	In Intelligent Robots and Systems, 2006 IEEE/RSJ
	International Conference on, pp. 5369-5375. IEEE, 2006.
 */

#include "anytimeRRT.h"
#include "validityChecker.h"
#include "myMotionValidator.h"
#include "costClass.h"
#include "draw.h"
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/MotionValidator.h>
#include <cmath>
#include <iostream>
#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;


void planWithSimpleSetup(int agent_num)
{
  // Construct the state space where we are planning

  ob::StateSpacePtr space(new ob::RealVectorStateSpace(2*agent_num));

  ob::RealVectorBounds bounds(2*agent_num);
  for(int j=0;j<agent_num;j++)
  {
	   //x dimension
	   bounds.setLow(2*j,0);
	   bounds.setHigh(2*j,1);

	   //y dimension
	   bounds.setLow(2*j+1,0);
	   bounds.setHigh(2*j+1,2.5);

  }


  space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

  // Instantiate SimpleSetup
  og::SimpleSetup ss(space);

  // Setup the StateValidityChecker
//  ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
  ompl::base::SpaceInformationPtr si=ss.getSpaceInformation();
  ss.setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si,agent_num,(bounds.high[0]-bounds.low[0])*0.01)));
 // ss.setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));

  si->setMotionValidator(ob::MotionValidatorPtr(new myMotionValidator(si,agent_num)));
  si->setup();


  // Setup Start and Goal
  ob::ScopedState<ob::RealVectorStateSpace> start(space);
  ob::ScopedState<ob::RealVectorStateSpace> goal(space);
 // goal->setXY(0.9,0.9);

  if(agent_num==3)
  {
	  start->values[0]=0.25;
	    start->values[1]=2.4;
	    start->values[2]=0.5;
	    start->values[3]=2.4;
	    start->values[4]=0.75;
	    start->values[5]=2.4;

	    goal->values[0]=0.25;
	    goal->values[1]=0.1;
	    goal->values[2]=0.5;
	    goal->values[3]=0.1;
	    goal->values[4]=0.75;
	    goal->values[5]=0.1;
  }
  else
  {
	  start->values[0]=0.25;
	  start->values[1]=2.4;
	  goal->values[0]=0.25;
	  goal->values[1]=0.1;
  }


//  start.random();
  std::cout << "start: "; start.print(std::cout);
//  goal.random();
  std::cout << "goal: "; goal.print(std::cout);

  ss.setStartAndGoalStates(start, goal);

  std::cout << "----------------" << std::endl;

  //ompl::base::PlannerPtr planner(new ompl::geometric::RRT(si));
//  ompl::geometric::RRT *my_rrt=new ompl::geometric::RRT(si);
  ompl::geometric::anytimeRRT *my_rrt=new ompl::geometric::anytimeRRT(si);
  my_rrt->setRange(0.03);
  my_rrt->setCostClass(new MyCostClass(si,agent_num));
  my_rrt->setup();
  ompl::base::PlannerPtr planner=ompl::base::PlannerPtr(my_rrt);

  ss.setPlanner(planner);
  si->setStateValidityCheckingResolution(0.02);
 // si->setMotionValidator(ob::MotionValidatorPtr(new myMotionValidator(si)));
  ss.setup();

  // Execute the planning algorithm
  ob::PlannerStatus solved = ss.solve(4);

  // If we have a solution,
  if (solved)
  {
    // Simplify the solution
	//ss.simplifySolution();
    std::cout << "----------------" << std::endl;
    std::cout << "Found solution:" << std::endl;
    // Print the solution path to screen

    ss.getSolutionPath().print(std::cout);

    // Print the solution path to a file
    std::ofstream ofs("path.dat");
    ss.getSolutionPath().printAsMatrix(ofs);
    ofs.close();
  }
  else
    std::cout << "No solution found" << std::endl;


  /*
   * if want to plot the path, use python to plot it. for this simple 2d case:
import numpy;
import matplotlib.pyplot as plt;
data=numpy.loadtxt('path.dat');
plt.plot(data[:,0],data[:,1],'.-');
plt.show();
plt.plot(data[:,0],data[:,1]);
plt.show();
   *
   */
}


int main()
{
	int agent_num=1;
  	std::cout<<"please input the number of agents: 1 or 3 ?"<<std::endl;
  	std::cin>>agent_num;
 // 	std::cout<<"default: using 3 agents"<<std::endl;
  	planWithSimpleSetup(agent_num);
  	if(agent_num==1 || agent_num==3) drawMap(1.0,2.5,agent_num);
  	return 0;
}




