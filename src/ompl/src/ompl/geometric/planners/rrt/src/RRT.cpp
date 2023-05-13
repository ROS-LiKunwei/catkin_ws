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

#include "ompl/geometric/planners/rrt/RRT.h"
#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

ompl::geometric::RRT::RRT(const base::SpaceInformationPtr &si, bool addIntermediateStates)
  : base::Planner(si, addIntermediateStates ? "RRTintermediate" : "RRT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &RRT::setRange, &RRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RRT::setGoalBias, &RRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RRT::setIntermediateStates, &RRT::getIntermediateStates,
                                "0,1");

    addIntermediateStates_ = addIntermediateStates;
}

ompl::geometric::RRT::~RRT()
{
    freeMemory();
}

void ompl::geometric::RRT::clear()
{
    Planner::clear();//调用基类 Planner 的 clear() 函数，清空所有成员变量。
    sampler_.reset();//调用智能指针 sampler_ 的 reset() 函数，将其指向的对象释放并设为 nullptr。
    freeMemory();//释放运动树中所有的节点以及节点对应的状态内存。
    if (nn_)
        nn_->clear();//如果最近邻数据结构 nn_ 存在，调用其 clear() 函数，清空所有存储的运动树节点。
    lastGoalMotion_ = nullptr;//将上一次找到的目标节点 lastGoalMotion_ 设为 nullptr。
}

void ompl::geometric::RRT::setup()
{
    //调用planner基类的setup函数
    Planner::setup();
    // 创建一个自我配置工具对象，用于配置规划器范围
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);


    // 如果最近邻数据结构尚未创建，则创建默认最近邻数据结构
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    // 设置最近邻数据结构的距离函数
    /*因为它捕获了在其定义之外声明的变量。它被称为闭包，因为它“封闭”了其捕获的变量，并在其内部函数中使用。
    闭包通常被用来创建一些可以在程序中传递和使用的功能单元。由于闭包封装了捕获的变量，所以它可以在任何需要该功能的地方被调用，并且可以使用它所捕获的变量。*/
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });//Motion* a和Motion* b并不是直接在setup()函数中传入的参数，而是在setDistanceFunction()中使用lambda表达式定义的闭包，其中的a和b将在后续调用nn_->nearest()等函数时传入。具体来说，setDistanceFunction()会将距离计算函数保存在最近邻数据结构中，在需要计算两个Motion之间距离时，最近邻数据结构（nn_）会调用这个函数并传入两个Motion指针，即a和b。
}

void ompl::geometric::RRT::freeMemory()
{
    if (nn_)//如果最近邻数据结构对象存在
    {
        std::vector<Motion *> motions;
        nn_->list(motions);// 获取所有保存在最近邻数据结构中的Motion对象，并存储到motions vector中
        for (auto &motion : motions)//遍历所有motion对象
        {
            if (motion->state != nullptr)//如果motion中的state指针被分配了内存
                si_->freeState(motion->state);//释放这个state的内存空间
            delete motion;//删除当前motion对象
        }
    }
}
/*
这段代码实现的是RRT(Rapidly-Exploring Random Tree)路径规划算法的solve函数，主要用于规划从起始状态到目标状态的路径。以下是它的实现步骤：

1.将起始状态添加到RRT的树结构中。

2.从起始状态开始，在状态空间中采样一个状态，然后找到树中距离它最近的状态。

3.如果找到的状态距离采样的状态小于一个预先设定的距离，那么直接将这两个状态连接在一起。

4.如果距离大于设定的距离，则需要在这两个状态之间插值，得到一系列连续的中间状态，然后将这些中间状态逐个加入到树中，最终将采样的状态加入到树中。

5.重复以上过程，直到找到一条连接起始状态和目标状态的路径，或者达到停止条件为止。

在实现过程中，还需要注意检查路径规划器是否有效，获取规划问题的目标区域并转换为可采样的目标区域等。
*/
ompl::base::PlannerStatus ompl::geometric::RRT::solve(const base::PlannerTerminationCondition &ptc)//输入参数是一个ompl::base::PlannerTerminationCondition对象，用于控制路径规划器的运行时间或其它停止条件。
{
    checkValidity(); //用于检查给定状态是否是有效的函数，其中 "有效" 是指它是否满足所有限制和约束条件。

    // 获取规划问题的目标区域并转换为可采样的目标区域。
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);//dynamic_cast 是 C++ 中的一个类型转换运算符，用于在运行时将指针或引用转换为另一种类型的指针或引用。它会在转换过程中检查类型是否有效，如果无效则返回 nullptr。

    // 1.初始化树。将起始状态添加到RRT的树结构中。这段代码是从 PlannerInputStates (pis_) 中获取起始状态，并将这些状态添加到 RRT 树的节点中。
    while (const base::State *st = pis_.nextStart())//pis_ 是一个 PlannerInputStates 类型的对象，其中包含了规划问题中的所有起始状态;nextStart() 是 PlannerInputStates 类的一个成员函数，它会返回一个 base::State 指针，指向下一个起始状态，如果所有起始状态都已经被使用过，那么它会返回一个空指针
    {
        // 首先用 new 运算符动态分配一个 Motion 对象，用于存储一个 RRT 树节点
        auto *motion = new Motion(si_);
        // 使用 si_->copyState() 函数将当前的起始状态拷贝到这个节点的状态中
        si_->copyState(motion->state, st);
        // nn_->add() 函数将这个节点添加到 RRT 树中去
        nn_->add(motion);
    }
    
    // 如果没有有效的起始状态，输出错误信息并返回无效状态。
    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    //如果没有定义采样器，就分配一个状态采样器。
    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;//存储找到的最佳路径
    Motion *approxsol = nullptr;//最近的路径
    double approxdif = std::numeric_limits<double>::infinity();//路径长度的近似值,初始化为正无穷
    auto *rmotion = new Motion(si_);//存储将要采样的状态
    base::State *rstate = rmotion->state;//状态空间的随机采样点
    base::State *xstate = si_->allocState();//临时状态（为一个状态分配内存）

    // 2.plan
    // 寻找一个满足条件的节点nmotion
    while (!ptc)
    {
        /*2.从起始状态开始，在状态空间中采样一个状态，然后找到树中距离它最近的状态。*/
        /* sample random state (with goal biasing) 
            带有goal biasing的随机状态采样
            这个条件语句会在 goalBias_ 的概率下从目标空间 goal_s 中采样一个状态，否则从整个状态空间 sampler_ 中采样一个状态    */
            //goal_s != nullptr目标状态空间指针非空，即存在目标状态空间；
            // 在0到1之间采样一个随机数，如果小于目标偏置（goal bias）的值，那么就使用目标状态空间进行采样
            // 目标状态空间可以进行采样
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);//目标状态空间采样
        else
            sampler_->sampleUniform(rstate);//普通的均匀采样

        /* find closest state in the tree 
            这行代码会在树中查找最近的节点 nmotion，其中 nn_ 是一个基于最近邻的数据结构        */
        Motion *nmotion = nn_->nearest(rmotion);
        // 这行代码将 dstate 初始化为 rstate
        base::State *dstate = rstate;

        /*3.如果找到的状态距离采样的状态小于一个预先设定的距离，那么直接将这两个状态连接在一起。*/
        /* find state to add  寻找要加入的状态*/
        double d = si_->distance(nmotion->state, rstate);//计算节点 nmotion 与采样的状态 rstate 之间的距离
        /* 4.如果距离大于设定的距离，则需要在这两个状态之间插值，得到一系列连续的中间状态，然后将这些中间状态逐个加入到树中，最终将采样的状态加入到树中。*/
        // 如果距离 d 大于 maxDistance_，那么树上就不能直接连接这两个状态，需要插入一个插值状态 xstate，并将其作为新的状态 dstate。
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }
        // 这行代码检查从节点 nmotion 到状态 dstate 的路径上是否存在碰撞。如果没有碰撞，则可以将 dstate 添加到树中。
        if (si_->checkMotion(nmotion->state, dstate))
        {
            // 如果需要添加插值状态，则生成从节点 nmotion 到 dstate 的插值状态，并将这些状态作为新的节点插入到树中。否则，直接将状态 dstate 添加到树中。
            if (addIntermediateStates_)
            {
                std::vector<base::State *> states;
                /* 获取有效的路径段数并加入树中*/
                //这行代码的作用是计算从一个状态到另一个状态的有效路径段数。在OMPL中，一个状态空间可能包含许多障碍物或约束条件，因此可能存在许多路径段无法满足这些限制。这行代码会计算从nmotion->state到dstate这一路径上的有效路径段数量。如果返回的count等于1，表示这两个状态之间没有障碍物，可以连接它们。如果返回的count大于1，表示这两个状态之间有障碍物，需要在它们之间采样更多的状态来构建可行路径。
                const unsigned int count = si_->getStateSpace()->validSegmentCount(nmotion->state, dstate);
                // 这段代码是在检查从当前运动到新生成的运动的路径上是否存在碰撞。如果路径有效，si_->getMotionStates函数将返回路径上的所有状态，并且这些状态将存储在states中。count表示这些状态的数量。如果路径无效，则函数将返回false，并且不会填充状态向量。在这种情况下，我们需要释放已经分配的状态，以避免内存泄漏和资源浪费。如果路径是有效的，由于第一个状态被传递给函数时已经被分配了，所以需要释放它，只保留最后一个状态。
                if (si_->getMotionStates(nmotion->state, dstate, states, count, true, true))
                    si_->freeState(states[0]);

                for (std::size_t i = 1; i < states.size(); ++i)
                {
                    auto *motion = new Motion;
                    motion->state = states[i];
                    motion->parent = nmotion;
                    nn_->add(motion);

                    nmotion = motion;
                }
            }
            else
            {
                // 一个节点(motion)包括两部分：parent指针和state
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, dstate);
                motion->parent = nmotion;
                nn_->add(motion);//将新的节点加入树；nn_用于存储树上的节点以及对其进行查询操作。

                nmotion = motion;//nmotion--当前的树节点
            }
            // 5.重复以上过程，直到找到一条连接起始状态和目标状态的路径，或者达到停止条件为止。
            double dist = 0.0;
            // 判断当前树中最新的节点状态是否满足目标条件，如果满足则将当前节点的状态赋值给solution，并且跳出while循环。同时，将距离赋值给dist
            bool sat = goal->isSatisfied(nmotion->state, &dist);
            if (sat)
            {
                approxdif = dist;
                solution = nmotion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = nmotion;
            }
        }
    }

    /* RRT算法找到初始路径后，对于给定的起点和目标点，它将尝试生成一条从起点到目标点的路径*/
    bool solved = false;
    bool approximate = false;//用于表示是否已找到一条近似的解决方案。
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    // 如果找到了精确的解决方案，则进行下一步
    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;//以便在下次迭代中使用

        /* construct the solution path */
        std::vector<Motion *> mpath;//声明mpath变量，并创建一个Motion类型的向量。这个向量将用于保存路径中的每个节点（运动）
        // 遍历运动，直到到达起点
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path  设置solution path*/
        auto path(std::make_shared<PathGeometric>(si_));//声明path变量，并创建一个类型为PathGeometric的共享指针。PathGeometric类表示几何路径，并将存储在mpath向量中的运动状态添加到该路径中。
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());//将解决方案路径添加到路径定义中，并将approximate和approxdif值传递给路径定义。
        solved = true;
    }

    /*释放分配的指针*/
    si_->freeState(xstate);//释放分配给xstate的空间
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}

void ompl::geometric::RRT::getPlannerData(base::PlannerData &data) const
{
    //获取有关运动规划器当前运行的信息。 重复调用此函数将更新数据（仅进行添加）。 这有助于查看探索数据结构中的变化，例如在调用 solve() 之间（无需调用 clear()）。
    Planner::getPlannerData(data);//调用了基类Planner中的getPlannerData函数，将传入的参数对象data传递给它进行处理；

    //这一部分代码创建一个指向Motion类型对象的指针向量motions。接下来，检查指向NearestNeighbors类型对象的指针nn_是否已经初始化。
    // 如果nn_已经初始化，调用list函数把所有在树上的节点的指针存储在一个vector对象motions中
    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    // 这一行代码检查lastGoalMotion_是否为nullptr（空指针），如果不是，则将它的状态信息加入到data中的一个称为goal vertex（目标顶点）的对象中。
    // lastGoalMotion_表示最后一次发现的目标顶点。
    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    // 这一部分代码使用for循环遍历所有的motions对象。对于每个motion对象，检查它的父节点是否为nullptr，
    // 如果是，则将它的状态信息添加到data中的一个称为start vertex（起始顶点）的对象中；
    // 如果不是，则将其父节点和自身的状态信息添加到data中的一个称为edge（边）的对象中。
    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}
