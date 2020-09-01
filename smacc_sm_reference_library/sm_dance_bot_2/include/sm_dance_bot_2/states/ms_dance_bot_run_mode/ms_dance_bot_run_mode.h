#include <smacc/smacc.h>
namespace sm_dance_bot_2
{
// STATE DECLARATION
class MsDanceBotRunMode : public smacc::SmaccState<MsDanceBotRunMode, SmDanceBot2, StAcquireSensors>
{
public:
   using SmaccState::SmaccState;

// TRANSITION TABLE
   typedef mpl::list<
   
   Transition<EvGlobalError, MsDanceBotRecoveryMode> 
   
   >reactions;
};
}