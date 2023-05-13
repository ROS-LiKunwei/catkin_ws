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

/* Author: Ioan Sucan */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_RRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_RRT_

#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gRRT
           @par Short description
           RRT is a tree-based motion planner that uses the following
           idea: RRT samples a random state @b qr in the state space,
           then finds the state @b qc among the previously seen states
           that is closest to @b qr and expands from @b qc towards @b
           qr, until a state @b qm is reached. @b qm is then added to
           the exploration tree.
           @par External documentation
           J. Kuffner and S.M. LaValle, RRT-connect: An efficient approach to single-query path planning, in <em>Proc.
           2000 IEEE Intl. Conf. on Robotics and Automation</em>, pp. 995–1001, Apr. 2000. DOI:
           [10.1109/ROBOT.2000.844730](http://dx.doi.org/10.1109/ROBOT.2000.844730)<br>
           [[PDF]](http://ieeexplore.ieee.org/ielx5/6794/18246/00844730.pdf?tp=&arnumber=844730&isnumber=18246)
           [[more]](http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html)
        */

        /** \brief Rapidly-exploring Random Trees */
        class RRT : public base::Planner
        {
        public:
            /** \brief Constructor */
            RRT(const base::SpaceInformationPtr &si, bool addIntermediateStates = false);

            ~RRT() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

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

            /** \brief Return true if the intermediate states generated along motions are to be added to the tree itself
             *                如果要将沿运动生成的中间状态添加到树本身，则返回true
             */
            bool getIntermediateStates() const
            {
                return addIntermediateStates_;
            }

            /** \brief Specify whether the intermediate states generated along motions are to be added to the tree
             *                指定是否将沿运动生成的中间状态添加到树中
             * itself */
            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }

            /** \brief Set the range the planner is supposed to use.
             *                设置规划器将要被使用的范围
                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. 
                这个参数极大地影响了算法的运行时间。它表示要添加到运动树中的运动的最大长度。*/
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /** \brief Get the range the planner is using 
             *                获取规划器正在使用的范围
            */
            double getRange() const
            {
                return maxDistance_;
            }

            /** \brief Set a different nearest neighbors datastructure 
             *                设置一个不同的最近邻居数据结构
            */
           /*
                这是一个成员函数模板，其作用是设置使用的最近邻数据结构类型，并将其实例化为指定的模板参数类型NN<Motion*>
                作用：
                        这个函数可以让用户自定义最近邻数据结构的类型，用来替换默认的数据结构类型，以便更好地适应特定的问题。
                        例如，如果某个用户想使用KD-Tree来管理节点，他可以定义一个KD-Tree类型的最近邻数据结构类，
                        然后使用setNearestNeighbors函数将其设置为RRT规划器使用的最近邻数据结构。这样就能更好地利用KD-Tree的快速查询特性，提高规划效率。
           */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                // 如果当前使用的最近邻数据结构不为空且包含元素，则输出警告信息：调用setNearestNeighbors会清除所有的状态
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();//实例化指定的最近邻数据结构类型，并将其赋值给成员变量nn_。
                setup();//调用setup()函数对规划器进行设置
            }

            void setup() override;

        protected:
            /** \brief Representation of a motion  motion代表一个运动（节点），包括节点的状态和其它一些信息

                This only contains pointers to parent motions as we  only need to go backwards in the tree. 
                这只包含指向父运动的指针，因为我们只需要在树中向后走。 */
            class Motion
            {
            public:
                Motion() = default;//默认构造函数

                /** \brief Constructor that allocates memory for the state */
                /*构造函数，通过传入space information指针，分配内存给state成员变量。
                具体过程：
                        SpaceInformation类是一个抽象的概念，它提供了一个空间描述和采样算法，以及可选的碰撞检测器、路径约束器等等。
                        allocState()方法会从SpaceInformation中获取一个合法的状态（使用采样器采样），并分配内存空间用于保存该状态，最后返回该状态的指针。
                */ 
                
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                ~Motion() = default;//默认析构函数

                /** \brief The state contained by the motion */
                // 运动（节点）状态，base::State类型的指针
                base::State *state{nullptr};

                /** \brief The parent motion in the exploration tree */
                // * 探索树中当前节点的父节点，是一个Motion类型的指针
                Motion *parent{nullptr};
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) 
                                计算运动（节点）之间的距离（包含状态之间的实际距离）
            */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief State sampler 状态采样器 */
            base::StateSamplerPtr sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions 包含运动树的最近邻数据结构*/
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;//声明了一个名为nn_的智能指针，该指针可以指向类型为NearestNeighbors<Motion *>的对象，并采用了std::shared_ptr的内存管理机制。这意味着当没有任何指针引用该对象时，该对象将自动被销毁，以避免内存泄漏。

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{.05};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
            bool addIntermediateStates_;

            /** \brief The random number generator 
             *                随机数生成器
             */
            RNG rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation 
             *                最近的目标节点。用来计算PlannerData
            */
            Motion *lastGoalMotion_{nullptr};
        };
    }
}

#endif
