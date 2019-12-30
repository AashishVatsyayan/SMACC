#include <smacc/smacc.h>
namespace sm_dance_bot
{
namespace SS1
{

namespace sm_dance_bot
{
namespace radial_motion
{
//forward declaration for initial ssr
class SsrRadialRotate;
class SsrRadialReturn;
class SsrRadialEndPoint;
class SsrRadialLoopStart;
} // namespace radial_motion
} // namespace sm_dance_bot

using namespace sm_dance_bot::radial_motion;

struct SsRadialPattern1 : smacc::SmaccState<SsRadialPattern1, MsDanceBotRunMode, SsrRadialLoopStart>
{
public:
    using SmaccState::SmaccState;

    typedef mpl::list<

        // Expected event
        smacc::transition<EvLoopEnd<SsrRadialLoopStart>, StRotateDegrees1, ENDLOOP>
        // Keyboard event
        //smacc::transition<EvKeyPressN<CbDefaultKeyboardBehavior>, StRotateDegrees1>,
        //smacc::transition<EvKeyPressP<CbDefaultKeyboardBehavior>, StNavigateToWaypointsX>,

        // Error events
        //smacc::transition<smacc::EvTopicMessageTimeout<CbLidarSensor>, StAcquireSensors>,
        //smacc::transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>>
        >

        reactions;

    static constexpr int total_iterations() { return 20; }
    static constexpr float ray_angle_increment_degree() { return 360.0 / total_iterations(); }
    static constexpr float ray_length_meters() { return 3; }

    int iteration_count = 0;

    static void onDefinition()
    {
        //static_configure<OrObstaclePerception, CbLidarSensor>();
    }

    void onInitialize()
    {
    }
};
//forward declaration for the superstate
using SS = SsRadialPattern1;

#include <sm_dance_bot/states/radial_motion/ssr_radial_end_point.h>
#include <sm_dance_bot/states/radial_motion/ssr_radial_return.h>
#include <sm_dance_bot/states/radial_motion/ssr_radial_rotate.h>
#include <sm_dance_bot/states/radial_motion/ssr_radial_loop_start.h>
} // namespace SS1
} // namespace sm_dance_bot