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
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

ompl::geometric::anytimeRRT::anytimeRRT(const base::SpaceInformationPtr &si) : base::Planner(si, "anytimeRRT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    goalBias_=0.05;
    maxDistance_=0.0;
    lastGoalMotion_=NULL;

    cost_bound=100000;//infinity
    current_cost=100000;//infinity
    dist_bias = 1.0;
    cost_bias = 0.0;
    delta_d=0.1;
    delta_c=0.1;
    epsilon_f=0.04;

    Planner::declareParam<double>("range", this, &anytimeRRT::setRange, &anytimeRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &anytimeRRT::setGoalBias, &anytimeRRT::getGoalBias, "0.:.05:1.");
}

ompl::geometric::anytimeRRT::~anytimeRRT()
{
    freeMemory();
}

void ompl::geometric::anytimeRRT::clear()
{
    Planner::clear();
    sampler_.reset();


    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = NULL;
}

void ompl::geometric::anytimeRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
    nn_->setDistanceFunction(boost::bind(&anytimeRRT::distanceFunction, this, _1, _2));
}

void ompl::geometric::anytimeRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::anytimeRRT::solve(const base::PlannerTerminationCondition &ptc)
{

	checkValidity();

	base::Goal *goal = pdef_->getGoal().get();
	base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

	bool solved = false;
	bool approximate = false;

	/*the following two variables are used to control the "time" of every RRT attempt*/
	bool first_rrt=true;
	int max_nodes_num=100000;//have not got a result after expending more than max_nodes_num nodes in the tree, then restart.
	//And after first rrt, let max_nodes_num=3*(num of nodes in the first rrt)

	base::State * start_state;
	start_state=si_->allocState();
	start_state=pdef_->getStartState(0);

	base::State * goal_state;
	goal_state=si_->allocState();
	if (goal_s && goal_s->canSample())
	    goal_s->sampleGoal(goal_state);


    while (ptc == false)
    {//for all RRT attempts
    	clear();

    	while (const base::State *st = pis_.nextStart()) //pis_ is a member in base::planner, and it is PlannerInputState, and PlannerInputState is an inner class in base::planner
    	{

    	    Motion *motion = new Motion(si_);
    	    si_->copyState(motion->state, st);
    	    nn_->add(motion);
    	}

    	if (nn_->size() == 0)
    	{
    	    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    	    return base::PlannerStatus::INVALID_START;
    	}

    	if (!sampler_)
    	    sampler_ = si_->allocStateSampler();

   // 	OMPL_INFORM("%s: Starting planning with %u states already in data structure", getName().c_str(), nn_->size());

    	Motion *solution  = NULL;
    	Motion *approxsol = NULL;
    	double  approxdif = std::numeric_limits<double>::infinity();
    	Motion *rmotion   = new Motion(si_);
    	base::State *rstate = rmotion->state;
    	base::State *xstate = si_->allocState();

    	while (ptc == false)
    	{//growing RRT until finding the goal in one RRT attempt

    		/* sample random state (with goal biasing) */
    		while(ptc == false)
    		{//sampling one satisfied state(qtarget)
    			int attempts=0;
    			if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
    			{
    				goal_s->sampleGoal(rstate);
    			}
    			else
    			{
    			  do{
    			    	 attempts++;
    			    	 sampler_->sampleUniform(rstate);
    			    }while(attempts<10 && (si_->distance(start_state, rstate)+si_->distance(rstate,goal_state)>cost_bound) && ptc == false);
    			}//suppose the max attempts num is 10;
    			if(attempts!=10) break;
    		}


    	    /* find closest k states in the tree */
    	    int K_neighbors=nn_->size()<5?nn_->size():5;// k=5
    	    std::vector<Motion *> Kmotion;
    	    nn_->nearestK(rmotion,K_neighbors,Kmotion);
    	    base::State *dstate = rstate;

    	    while(ptc == false && Kmotion.size()>0)
    	    {//try for every neighbor
    	    	int min_cost_index;
    	    	min_cost_index=0;
    	    	double min_cost=selCost(Kmotion.at(0),rmotion);
    	    	for(int i=1;i<Kmotion.size();i++)
    	    	{
    	    		if(selCost(Kmotion.at(i),rmotion)<min_cost)
    	    		{
    	    			min_cost=selCost(Kmotion.at(i),rmotion);
    	    			min_cost_index=i;
    	    		}
    	    	}

    	    	double d = si_->distance(Kmotion.at(min_cost_index)->state, rstate);
    	    	if (d > maxDistance_)
    	    	{
    	    	    si_->getStateSpace()->interpolate(Kmotion.at(min_cost_index)->state, rstate, maxDistance_ / d, xstate);
    	    	    dstate = xstate;
    	    	}

    	    	double estimate_cost=Kmotion.at(min_cost_index)->cost+si_->distance(Kmotion.at(min_cost_index)->state, dstate)+si_->distance(dstate, goal_state);
    	    	if(estimate_cost<=cost_bound)
    	    	{
    	    	    if (si_->checkMotion(Kmotion.at(min_cost_index)->state, dstate))
    	    	    {
    	    	    	/* create a motion */
    	    	    	Motion *motion = new Motion(si_);
    	    	    	si_->copyState(motion->state, dstate);//if qtarget is the goal, then goal will be added to the tree here
    	    	    	motion->parent = Kmotion.at(min_cost_index);
    	    	    	motion->cost=motion->parent->cost+costClassPtr->cost(motion->parent->state,motion->state);//si_->distance(motion->parent->state,motion->state);


    	    	    	nn_->add(motion);
    	    	    	double dist = 0.0;
    	    	    	bool sat = goal_s->isSatisfied(motion->state, &dist);
    	    	    	if (sat)
    	    	    	{
    	    	    	    approxdif = dist;
    	    	    	    solution = motion;
    	    	    	    goto out1;
    	    	    	}
    	    	    	if (dist < approxdif)
    	    	    	{
    	    	    	    approxdif = dist;
    	    	    	    approxsol = motion;
    	    	    	}
    	    	    	if(nn_->size()>max_nodes_num) goto out1;//restart

    	    	    	break;
    	    	    }

    	    	    else
    	    	    {
    	    	        Kmotion.erase(Kmotion.begin()+min_cost_index);//current cost-minimum node is not worth extension, remove it.
    	    	    }

    	    	}

    	    	else
    	    	{
    	    		Kmotion.erase(Kmotion.begin()+min_cost_index);//current cost-minimum node is not worth extension, remove it.
    	    	}
    	    }

    	}

out1: 	if (solution == NULL)
    	{
    	  //  solution = approxsol;
    	  //  approximate = true;
    	}

    	if (solution != NULL)
    	{
    	    lastGoalMotion_ = solution;

    	    /* construct the solution path */
    	    std::vector<Motion*> mpath;
    	    while (solution != NULL)
    	    {
    	        mpath.push_back(solution);
    	        solution = solution->parent;
    	    }

    	    /* set the solution path */
    	    PathGeometric *path = new PathGeometric(si_);
    	    for (int i = mpath.size() - 1 ; i >= 0 ; --i)
    	        path->append(mpath[i]->state);
    	    pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif, getName());
    	    solved = true;

    	    current_cost=lastGoalMotion_->cost;//the last motion is goal motion, so lastGoalMotion_->cost is the cost from start to goal
    	    std::cout<<"cost: "<<current_cost<<std::endl;
    	    cost_bound=(1-epsilon_f)*current_cost;
    	    //std::cout<<"cost_bound:"<<cost_bound<<std::endl;
    	    dist_bias=dist_bias-delta_d;
    	    cost_bias=cost_bias+delta_c;
    	    if(dist_bias<0) dist_bias=0;
    	    if(cost_bias>1) cost_bias=1.0;

    	    if(first_rrt)
    	    {
    	    	max_nodes_num=3*nn_->size();
    	    	first_rrt=false;
    	    }
    	}

    	si_->freeState(xstate);
    	if (rmotion->state)
    	    si_->freeState(rmotion->state);
    	delete rmotion;

    }

    //if(start_state)si_->freeState(start_state);
    //if(goal_state)si_->freeState(goal_state);

    OMPL_INFORM("%s: Created %u states in the final RRT", getName().c_str(), nn_->size());

    return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::anytimeRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->parent == NULL)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
        else
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state),
                         base::PlannerDataVertex(motions[i]->state));
    }
}
