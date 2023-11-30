/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, University of Toronto
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
*   * Neither the name of the University of Toronto nor the names of its
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

/* Authors: Louis Petit */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_INFORMED_RRTROPE_
#define OMPL_GEOMETRIC_PLANNERS_RRT_INFORMED_RRTROPE_

#include "ompl/geometric/planners/rrt/RRTConnect.h"

namespace ompl
{
    namespace geometric
    {
        /**
             @anchor gRRTRope
           @par Short description
           The basic idea is to find a feasible path quickly 
           with an altered version of "RRT-connect", and shotcut it
           by looking at the fartherst points first.
           @par External documentation
           L. Petit and A. L. Desbiens, RRT-Rope: A deterministic shortening approach for fast near-optimal path planning in large-scale uncluttered 3D environments,
           in <em>2021 IEEE International Conference on Systems, Man, and Cybernetics (SMC)</em>, Melbourne, Australia, 2021, pp. 1111-1118. DOI: 
           [10.1109/SMC52423.2021.9659071](http://dx.doi.org/10.1109/SMC52423.2021.9659071)<br>
           [[PDF]](https://www.researchgate.net/publication/357636884_RRT-Rope_A_deterministic_shortening_approach_for_fast_near-optimal_path_planning_in_large-scale_uncluttered_3D_environments)
           [[more]](https://www.edu.louispetit.be/projects/rrt-rope)
           <a href="https://www.youtube.com/watch?v=zd54WoifAIk&ab_channel=LouisPetit">Short description video</a>.
        */

        /** \brief RRT-Rope (RRTRope) */
        class RRTRope : public RRTConnect
        {
        public:
            /** \brief Constructor */
            RRTRope(const base::SpaceInformationPtr &si, double delta = 1.0);

            /** \brief Set the step size (delta) the planner uses internally. This parameter also influences the runtime of the
                algorithm. */
            void setDelta(double delta)
            {
                delta_ = delta;
            }

            /** \brief Get the delta value the planner is using */
            double getDelta() const
            {
                return delta_;
            }

        protected:
            /** \brief The step size to use during intermediate node insertion and shortcutting */
            double delta_{1.0};
            
        };
    }
}

#endif
