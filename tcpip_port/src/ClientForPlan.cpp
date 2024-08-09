#include <car_msgs/CarCmd.h>
#include <tcpip_port.h>
#include <ros/ros.h>

tcpip_port port;
port.initial(9998);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_cmd_client");
    ros::NodeHandle nh;
    ros::Subscriber mpc_cmd_client = nh.subscribe<car_msgs::CarCmd>("car_cmd");

    port.accept_client();
    while(ros::ok())
    {
        int flag = port.recvData();
        if(flag > 0)
        {
            
            planeMapping::Timer time1;
            time1.reset();
            port.analysisBuf();
            srv.request.isNeedFootStep = 1;
            srv.request.indexLast      = 1;
            srv.request.isLeftLast     = 1;
            srv.request.restart = 0;
            for(const auto & ch : port.date_receive.flag)
            {
                std::cout << " 收到的标志符为：" << ch << std::endl;
            }
            // 控制端发3个：IT，CE，SP
            std::cout << "reques data copy finish" << std::endl;

            if(port.date_receive.flag[0] == 'I' && port.date_receive.flag[1] == 'T')
            {
                countPlan = 0;
                srv.request.restart = 1;
            }
            else if(port.date_receive.flag[0] == 'C' && port.date_receive.flag[1] == 'E')
            {
                ;
            }
            else if(port.date_receive.flag[0] == 'S' && port.date_receive.flag[1] == 'P')
            {
                isStop = 2;
            }
            else
            {
                std::cout << "无效字符" <<std::endl;
                isStop = 2;
            }

            bool plan_flag = foot_plan_client.call(srv);
            if(plan_flag)
            {
                //先检查footstep_plan的状态
                if((int32_t)srv.response.state_plan == (int32_t)0)
                {
                    ROS_INFO("Planner is not prepared for planning!-----------");
                    continue;
                }
                else if((int32_t)srv.response.state_plan == (int32_t)1)
                {
                    ROS_INFO("Planning complete! Please stop!!!-----------");
                    //在这里添加停止发送数据的标志
                    isStop ++;
                }
                else if(srv.response.state_plan == 2)
                {
                    ROS_INFO("Planning successually!-----------");
                    // int index = 0;
                    // for(const auto & fp : srv.response.footstepsResult.footsteps)
                    // {
                        // std::cout << index << ": " << fp.x << ", " << fp.y << ", " << fp.z << ", " << fp.roll << ", " << fp.pitch
                        // << fp.yaw << ", " << fp.isLeft << std::endl;
                    //     index ++;
                    // }
                }
                else
                {
                    ROS_INFO("No valide result received!-----------");
                    continue;
                }


                // 这里，srv.response.footstepsResult.footsteps就其实就是和控制端那里添加之后的步子一样的
                // 所以在这里判断步态的左右脚是否交叉


                int dataindex = 1;
                // 此时是stop，双脚站立，从第0步都发出去；否则，第1步不发，因为是重复的，控制端保留了一步

                std::vector<planeMapping::PathType> stepsForSend;
                for (size_t i = 0; i < srv.response.footstepsResult.footsteps.size(); i++)
                {
                    planeMapping::PathType step;
                    step.isLeft = srv.response.footstepsResult.footsteps[i].isLeft;
                    step.Pos[0]       = srv.response.footstepsResult.footsteps[i].x;
                    step.Pos[1]       = srv.response.footstepsResult.footsteps[i].y;
                    step.Pos[2]       = srv.response.footstepsResult.footsteps[i].z;
                    step.Ori[0]       = srv.response.footstepsResult.footsteps[i].yaw / 57.3;
                    step.Ori[1]       = srv.response.footstepsResult.footsteps[i].pitch / 57.3;
                    step.Ori[2]       = srv.response.footstepsResult.footsteps[i].roll /57.3;
                    step.type         = srv.response.footstepsResult.footsteps[i].type;
                    step.stairEdge1Pos[0]  = srv.response.footstepsResult.footsteps[i].stairedgepoint1;
                    step.stairEdge1Pos[1]  = srv.response.footstepsResult.footsteps[i].stairedgepoint2;
                    step.stairEdge1Pos[2]  = srv.response.footstepsResult.footsteps[i].stairedgepoint3;
                    step.stairEdge2Pos[0]  = 0.0;
                    step.stairEdge2Pos[1]  = 0.0;
                    step.stairEdge2Pos[2]  = 0.0;
                    stepsForSend.emplace_back(step);
                }
                std::vector<int> errorIndex = checkFootSteps(stepsForSend);
                if(errorIndex.size() == 2)
                {
                    isStop = 2;//直接停止
                    ROS_WARN("\033[1;32m Two errorIndex\033[Om"); 
                }
                //如果有一个地方出现问题
                else if(errorIndex.size() == 1)
                {
                    ROS_WARN("\033[1;32m One errorIndex\033[Om"); 
                    //那么就反转一下
                    isStop = 2;//直接停止
                    for(int i = errorIndex[0]; i < stepsForSend.size(); i++)
                    {
                        if(stepsForSend[i].isLeft == 1)
                        {
                            stepsForSend[i].isLeft = 0;
                        }
                        else
                        {
                            stepsForSend[i].isLeft = 1;
                        }
                        stepsForSend[i].Pos[1] *= -1;
                    }
                }
                if(countPlan != 0)
                {
                    stepsForSend.erase(stepsForSend.begin());
                }
                if(countPlan == 0)
                {
                    stepsForSend[0].Pos[2] = 0.0;
                }
                // isStop要变成2就不发送了
                if(isStop >= 2)
                {
                    stepsForSend.resize(0);
                }
                if(countPlan >= 2 && stepsForSend[0].isLeft == 1)
                {
                    leftCatogroyCurrent = stepsForSend[0].type;
                    if(leftCatogroyLast == 2 && leftCatogroyCurrent == 4)
                    {
                        stepsForSend[0].Pos[2] += 0.01;
                        std::cout << "加了偏量1cm" << std::endl;
                    }
                }
                if(countPlan >= 2 && stepsForSend[0].isLeft == 0)
                {
                    rightCatogroyCurrent = stepsForSend[0].type;
                    if(rightCatogroyLast == 2 && rightCatogroyCurrent == 4)
                    {
                        stepsForSend[0].Pos[2] += 0.01;
                        std::cout << "加了偏量1cm" << std::endl;
                    }
                }
                leftCatogroyLast = leftCatogroyCurrent;
                rightCatogroyLast = rightCatogroyCurrent;

                port.sendSteps(stepsForSend);
                countPlan ++;
                std::cout << "耗时：" << time1.elapsed() << " ms" << std::endl;
            }
            else
            {
                ROS_ERROR("client failed");
                port.close_client();
                port.closetcpip();
                return 1;
            }
        }
        else
        {
            port.close_client();
            port.closetcpip();
            std::cout << "收到的data不对" << std::endl;
            return 0;
        }
        // port.close_client();
    }
    port.close_client();
    std::cout << "正常结束一次数据传输" << std::endl;
    port.closetcpip();
    return 0;
}

