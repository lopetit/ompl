/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* \author Ioan Sucan */

#include "ompl/kinematic/planners/rrt/LazyRRT.h"
#include "ompl/base/GoalSampleableRegion.h"
#include <cassert>

bool ompl::kinematic::LazyRRT::solve(double solveTime)
{
    SpaceInformationKinematic  *si     = dynamic_cast<SpaceInformationKinematic*>(m_si); 
    base::Goal                 *goal   = m_pdef->getGoal();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);
    unsigned int                   dim = si->getStateDimension();
    
    if (!goal)
    {
	m_msg.error("LazyRRT: Goal undefined");
	return false;
    }

    time::point endTime = time::now() + time::seconds(solveTime);
    
    if (m_nn.size() == 0)
    {
	for (unsigned int i = 0 ; i < m_pdef->getStartStateCount() ; ++i)
	{
	    Motion *motion = new Motion(dim);
	    si->copyState(motion->state, m_pdef->getStartState(i));
	    if (si->satisfiesBounds(motion->state) && si->isValid(motion->state))
	    { 
		motion->valid = true;
		motion->root = m_pdef->getStartState(i);
		m_nn.add(motion);
	    }	
	    else
	    {
		m_msg.error("LazyRRT: Initial state is invalid!");
		delete motion;
	    }	
	}
    }
    
    if (m_nn.size() == 0)
    {
	m_msg.error("LazyRRT: There are no valid initial states!");
	return false;	
    }    

    m_msg.inform("LazyRRT: Starting with %u states", m_nn.size());

    std::vector<double> range(dim);
    for (unsigned int i = 0 ; i < dim ; ++i)
	range[i] = m_rho * (si->getStateComponent(i).maxValue - si->getStateComponent(i).minValue);
    
    Motion *solution = NULL;
    double  distsol  = -1.0;
    Motion *rmotion  = new Motion(dim);
    base::State *rstate = rmotion->state;
    base::State *xstate = new base::State(dim);

 RETRY:

    while (time::now() < endTime)
    {
	/* sample random state (with goal biasing) */
	if (goal_s && m_rng.uniform01() < m_goalBias)
	    goal_s->sampleGoal(rstate);
	else
	    m_sCore->sample(rstate);

	/* find closest state in the tree */
	Motion *nmotion = m_nn.nearest(rmotion);
	assert(nmotion != rmotion);
	
	/* find state to add */
	for (unsigned int i = 0 ; i < dim ; ++i)
	{
	    double diff = rmotion->state->values[i] - nmotion->state->values[i];
	    xstate->values[i] = fabs(diff) < range[i] ? rmotion->state->values[i] : nmotion->state->values[i] + diff * m_rho;
	}
	
	/* create a motion */
	Motion *motion = new Motion(dim);
	si->copyState(motion->state, xstate);
	motion->parent = nmotion;
	nmotion->children.push_back(motion);
	motion->root = nmotion->root;
	m_nn.add(motion);
	
	double dist = 0.0;
	if (goal->isSatisfied(motion->state, motion->root, &dist))
	{
	    distsol = dist;
	    solution = motion;
	    break;
	}
    }
    
    if (solution != NULL)
    {
	/* construct the solution path */
	std::vector<Motion*> mpath;
	while (solution != NULL)
	{
	    mpath.push_back(solution);
	    solution = solution->parent;
	}
	
	/* check the path */
	for (int i = mpath.size() - 1 ; i >= 0 ; --i)
	    if (!mpath[i]->valid)
	    {
		if (si->checkMotion(mpath[i]->parent->state, mpath[i]->state))
		    mpath[i]->valid = true;
		else
		{
		    removeMotion(mpath[i]);
		    goto RETRY;
		}
	    }
	
	/*set the solution path */
	PathKinematic *path = new PathKinematic(m_si);
	for (int i = mpath.size() - 1 ; i >= 0 ; --i)
	{
	    base::State *st = new base::State(dim);
	    si->copyState(st, mpath[i]->state);
	    path->states.push_back(st);
	}
	
	goal->setDifference(distsol);
	goal->setSolutionPath(path);	
	
    }
    
    delete xstate;
    delete rmotion;
    
    m_msg.inform("LazyRRT: Created %u states", m_nn.size());

    return goal->isAchieved();
}

void ompl::kinematic::LazyRRT::removeMotion(Motion *motion)
{
    m_nn.remove(motion);
    
    /* remove self from parent list */
    
    if (motion->parent)
    {
	for (unsigned int i = 0 ; i < motion->parent->children.size() ; ++i)
	    if (motion->parent->children[i] == motion)
	    {
		motion->parent->children.erase(motion->parent->children.begin() + i);
		break;
	    }
    }    

    /* remove children */
    for (unsigned int i = 0 ; i < motion->children.size() ; ++i)
    {
	motion->children[i]->parent = NULL;
	removeMotion(motion->children[i]);
    }
}

void ompl::kinematic::LazyRRT::getStates(std::vector<const base::State*> &states) const
{
    std::vector<Motion*> motions;
    m_nn.list(motions);
    states.resize(motions.size());
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
	states[i] = motions[i]->state;
}
