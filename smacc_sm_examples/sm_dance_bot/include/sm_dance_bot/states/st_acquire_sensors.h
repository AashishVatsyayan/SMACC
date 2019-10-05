using namespace smacc;

struct StAcquireSensors : 
      smacc::SmaccState<StAcquireSensors, SmDanceBot>
{
   using SmaccState::SmaccState;

   typedef mpl::list<
       
       // Expected event
       sc::transition<EvStateFinish<StAcquireSensors>, StNavigateToWaypointsX>,
       
       // Keyboard event
       sc::transition<EvKeyPressN<SbKeyboard>, StNavigateToWaypointsX>,
       
       // Sensor events
       sc::custom_reaction<EvTopicMessage<LidarSensor>>,
       sc::custom_reaction<EvTopicMessage<smacc::SensorTopic<sensor_msgs::Temperature>>>
       >
       reactions;

   AllEventAggregator allSensorsReady;

   void onInitialize()
   {
      this->configure<ObstaclePerceptionOrthogonal>(std::make_shared<LidarSensor>());
      this->configure<SensorOrthogonal>(std::make_shared<SbConditionTemperatureSensor>());
      this->configure<KeyboardOrthogonal>(std::make_shared<SbKeyboard>());

      this->configure<PublisherOrthogonal>(std::make_shared<SbStringPublisher>("Hello World!"));
      this->configure<Service3Orthogonal>(std::make_shared<Service3Behavior>(Service3Command::SERVICE3_ON));

      allSensorsReady.setTriggerEventTypesCount(2);
   }
   
   sc::result react(const EvTopicMessage<LidarSensor> &ev)
   {
      ROS_INFO_ONCE("Lidar sensor is ready");

      if (allSensorsReady.notify(ev))
         this->throwFinishEvent();

      return discard_event();
   }

   sc::result react(const EvTopicMessage<smacc::SensorTopic<sensor_msgs::Temperature>> &ev)
   {
      ROS_INFO_ONCE("Temperature sensor is ready");
      if (allSensorsReady.notify(ev))
         this->throwFinishEvent();

      return discard_event();
   }
};
