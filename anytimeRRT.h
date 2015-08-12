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

#ifndef ANYTIMERRT_H_
#define ANYTIMERRT_H_
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"


namespace ob = ompl::base;
namespace og = ompl::geometric;

class CostClass
{public:
	CostClass(){}
	CostClass(ob::SpaceInformationPtr &si)
	{
		si_=si.get();
	}
	virtual double cost(ob::State* state1, ob::State * state2 )
	{//if we don't define our own customed cost class, it means the cost is uniform over the entire free space.
		//the anytimeRRT algorithm uses this CostClass as default. If you want to change the cost function, just
		//write a cost class inheriting from this class and call setCostClass() before using the anytimeRRT planner
		return si_->distance(state1,state2);
	}
	virtual ~CostClass(){}

	ob::SpaceInformation *si_;
};

namespace ompl
{

    namespace geometric
    {

        /** \brief Rapidly-exploring Random Trees */
        class anytimeRRT : public base::Planner
        {
        public:

            /** \brief Constructor */
            anytimeRRT(const base::SpaceInformationPtr &si);

            virtual ~anytimeRRT();

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear();

            /** \brief Set the goal bias

                In the process of randomly selecting states in
                the state space to attempt to go towards, the
                algorithm may in fact choose the actual goal state, if
                it knows it, with some probability. This probability
                is a real number between 0.0 and 1.0; its value should
                usually be around 0.05 and should not be too large. It
                is probably a good idea to use the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Set the range the planner is supposed to use.

                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. */
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /** \brief Get the range the planner is using */
            double getRange() const
            {
                return maxDistance_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template<template<typename T> class NN>
            void setNearestNeighbors()
            {
                nn_.reset(new NN<Motion*>());
            }

            virtual void setup();

        protected:


            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:

                Motion() : state(NULL), parent(NULL),cost(0.0)
                {
                }

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(NULL),cost(0.0)
                {
                }

                ~Motion()
                {
                }

                /** \brief The state contained by the motion */
                base::State       *state;

                double cost;

                /** \brief The parent motion in the exploration tree */
                Motion            *parent;

            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();


            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *qtree, const Motion *qtarget) const
            {
                return si_->distance(qtree->state,qtarget->state);
            }

            double heuristicCost(const Motion *a, const Motion *b)
            {//if it is in 2d space, use the straight-line distance as the heuristic cost of going from a to b
            	 return si_->distance(a->state, b->state);
            }

            double selCost(const Motion *qtree, const Motion *qtarget)
            {
                 return dist_bias*si_->distance(qtree->state, qtarget->state)+cost_bias*qtree->cost;
            }

        public:
            void setCostClass(CostClass* costPtr)
            {
            	costClassPtr=costPtr;
            }



            /** \brief State sampler */
            base::StateSamplerPtr                          sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            boost::shared_ptr< NearestNeighbors<Motion*> > nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
            double                                         goalBias_;

            /** \brief The maximum length of a motion to be added to a tree */
            double                                         maxDistance_;

            double cost_bound;//the total cost of previous RRT. used as cost bound in current RRT. (known as Cs in the paper)
            double current_cost;//total cost of current RRT.
            double dist_bias;//distance bias, known as T.db in the paper.
            double cost_bias;//cost bias, known as T.cb in the paper
            double delta_d,delta_c;//decreasing dist_bias by δd each iteration and increasing cost_bias by δc
            double epsilon_f;//each successive solution was guaranteed to be epsilon_f less costly than the previous solution

            CostClass*  costClassPtr;



            /** \brief The random number generator */
            RNG                                            rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion                                         *lastGoalMotion_;
        };

    }
}


#endif /* ANYTIMERRT_H_ */
