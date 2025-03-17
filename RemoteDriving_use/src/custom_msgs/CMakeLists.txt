cmake_minimum_required(VERSION 2.8.3)
project(custom_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp rospy
  std_msgs
  message_generation
  geometry_msgs
)

add_message_files(
    DIRECTORY msg/Controller
    FILES
    AEBCmd.msg
    GearCmd.msg
    SteeringCmd.msg
    TrqBreCmd.msg
    TrqBreCmd485.msg
    SteerRegine.msg
    )

add_message_files(
    DIRECTORY msg/Detection
    FILES
    ImageObject.msg
    LidarRawObject.msg
    Object.msg
    ImageObjectArray.msg
    LidarRawObjectArray.msg
    ObjectArray.msg
    RadarRawObject.msg
    RadarRawObjectArray.msg
    RadarObject.msg
    RadarObjectArray.msg
    ArsRadarRaw.msg
    ArsRadarRawArray.msg
    )

add_message_files(
    DIRECTORY msg/Localization
    FILES    
    CurPose.msg
    NaviData.msg
    SlamPose.msg
    UWBPose.msg
    ParkSpace.msg
    )
add_message_files(
    DIRECTORY msg/SmallCar
    FILES
    ControlMode.msg
    PointSelect.msg
    )
add_message_files(
    DIRECTORY msg/Map
    FILES
    LaneLine.msg
    LaneLineArray.msg
    )

add_message_files(
    DIRECTORY msg/Planning
    FILES
    Path.msg
    Request.msg
    AEB.msg
    )

add_message_files(
    DIRECTORY msg/VehicleStat
    FILES
    VehicleStat.msg
    )

add_message_files(
    DIRECTORY msg/Task
    FILES
    ImageDetect.msg
    RoadAttri.msg
    TaskAttri.msg
    distant.msg
    )

add_message_files(
    DIRECTORY msg/UWB
    FILES
    LinktrackAnchorframe0.msg
    LinktrackAoaNode0.msg
    LinktrackAoaNodeframe0.msg
    LinktrackNode0.msg
    LinktrackNode1.msg
    LinktrackNode2.msg
    LinktrackNodeframe0.msg
    LinktrackNodeframe1.msg
    LinktrackNodeframe2.msg
    LinktrackNodeframe3.msg
    LinktrackTag.msg
    LinktrackTagframe0.msg
    Test.msg
    TofsenseCascade.msg
    TofsenseFrame0.msg
    )
add_message_files(
    DIRECTORY msg/TurnLightStat
    FILES
    TurnLightStat.msg
    )
add_message_files(
    DIRECTORY msg/Camera_Status
    FILES
    Lane_Stat.msg
    Camera_Obj.msg
    Alert_Stat.msg
    Camera_Obj_Array.msg
    Cam_Obj.msg
    Fusion_Datas.msg
    Fusion_Datas_Array.msg
    CamObj.msg
    CamObjArray.msg
    )
add_message_files(
    DIRECTORY msg/Ars
    FILES
    Objects.msg
    ObjectExtended.msg
    ObjectGeneral.msg
    ObjectList.msg
    ObjectQuality.msg
    ObjectStatus.msg
    )
add_message_files(
    DIRECTORY msg/Fusion
    FILES
    RadarPixArray.msg
    RadarPix.msg
    )
#------------------------------------
add_service_files(
   DIRECTORY srv/V2X
    FILES
    Ambulance.srv
    TrafficLight.srv
    GateStat.srv
)

add_service_files(
   DIRECTORY srv/PLAN
    FILES
    Control.srv
    Route.srv
)

add_service_files(
   DIRECTORY srv/TASK
    FILES
    Task.srv
)

add_service_files(
   DIRECTORY srv/MAP_SW
    FILES
    Map_Switch.srv
)

add_service_files(
   DIRECTORY srv/WARN
    FILES
    Warn.srv
)

add_service_files(
   DIRECTORY srv/APP
    FILES
    App.srv
)

add_service_files(
   DIRECTORY srv/LIDAR
    FILES
    Wharf.srv
)

add_service_files(
   DIRECTORY srv/BCM
    FILES
    DoorCmd.srv
    FogLampCmd.srv
    HornsCmd.srv
    BrakeLightCmd.srv
    SideLampsCmd.srv
    TurnLightCmd.srv
    WindowCmd.srv
    WipersCmd.srv
    EPBCmd.srv
    LKSStatusCmd.srv
    LowHeadLightCmd.srv
)

#--------------------------------------
 generate_messages(
     DEPENDENCIES
     std_msgs
     geometry_msgs
 )

catkin_package(
CATKIN_DEPENDS
message_runtime
)
