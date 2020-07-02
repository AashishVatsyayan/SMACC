#include <smacc/smacc.h>
namespace sm_moveit_4
{
    // STATE DECLARATION
    struct StInitialForward : smacc::SmaccState<StInitialForward, SmMoveIt44>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<

            // Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StRotate180, SUCCESS>
            Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, SS1::SsPickObject, SUCCESS>,
            Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StInitialForward, ABORT>>
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            //configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>();
            configure_orthogonal<OrNavigation, CbNavigateForward>(1.2);
        }

        void runtimeConfigure()
        {
            ROS_INFO("runtime");
            sm_moveit_4::cl_perception_system::ClPerceptionSystem *perceptionSystem;
            this->requiresClient(perceptionSystem);

            for (auto &ci : perceptionSystem->cubeInfos_)
            {
            }
        }

        void OnEntry()
        {
            ROS_INFO("state on entry");
        }
    };
} // namespace sm_moveit_4