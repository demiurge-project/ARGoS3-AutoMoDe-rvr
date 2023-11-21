# Set up ARGoS compilation information
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})

set(BEHAVIOR_TREE_HEADERS
	core/AutoMoDeBehaviorTree.h
	core/AutoMoDeBehaviorTreeBuilder.h
	core/bt/Node.h
	core/bt/Selector.h
	core/bt/SequenceStar.h
	#AutoMoDe
	core/AutoMoDeRobotDAO.h
	core/AutoMoDeControllerBehaviorTree.h
	core/AutoMoDeRabBuffer.h
	# Behaviours
	modules/AutoMoDeBehaviour.h
	modules/AutoMoDeBehaviourAntiPhototaxis.h
	modules/AutoMoDeBehaviourPhototaxis.h
	modules/AutoMoDeBehaviourAttraction.h
	modules/AutoMoDeBehaviourRepulsion.h
	modules/AutoMoDeBehaviourStop.h
	modules/AutoMoDeBehaviourExploration.h
	# Conditions
	modules/AutoMoDeCondition.h
	modules/AutoMoDeConditionBlackFloor.h
	modules/AutoMoDeConditionWhiteFloor.h
	modules/AutoMoDeConditionGrayFloor.h
	modules/AutoMoDeConditionNeighborsCount.h
	modules/AutoMoDeConditionInvertedNeighborsCount.h
	modules/AutoMoDeConditionFixedProbability.h)


set(BEHAVIOR_TREE_SOURCES
	core/AutoMoDeBehaviorTree.cpp
	core/AutoMoDeBehaviorTreeBuilder.cpp
	core/bt/Node.cpp
	core/bt/Selector.cpp
	core/bt/SequenceStar.cpp
	#AutoMoDe
	core/AutoMoDeRobotDAO.cpp
	core/AutoMoDeControllerBehaviorTree.cpp
	core/AutoMoDeRabBuffer.cpp
	# Behaviours
	modules/AutoMoDeBehaviour.cpp
	modules/AutoMoDeBehaviourAntiPhototaxis.cpp
	modules/AutoMoDeBehaviourPhototaxis.cpp
	modules/AutoMoDeBehaviourAttraction.cpp
	modules/AutoMoDeBehaviourRepulsion.cpp
	modules/AutoMoDeBehaviourStop.cpp
	modules/AutoMoDeBehaviourExploration.cpp
	# Conditions
	modules/AutoMoDeCondition.cpp
	modules/AutoMoDeConditionBlackFloor.cpp
	modules/AutoMoDeConditionWhiteFloor.cpp
	modules/AutoMoDeConditionGrayFloor.cpp
	modules/AutoMoDeConditionNeighborsCount.cpp
	modules/AutoMoDeConditionInvertedNeighborsCount.cpp
	modules/AutoMoDeConditionFixedProbability.cpp)

add_executable(automode_bt
	/home/aligot/Desktop/Arena/iridia-tracking-system/src/plugins/robots/e-puck/real_robot/real_rvr_its.h
	/home/aligot/Desktop/Arena/iridia-tracking-system/src/plugins/robots/e-puck/real_robot/real_rvr_its_main.cpp
	${BEHAVIOR_TREE_HEADERS} ${BEHAVIOR_TREE_SOURCES})

target_link_libraries(automode_bt
	argos3plugin_${ARGOS_BUILD_FOR}_rvr
	argos3plugin_${ARGOS_BUILD_FOR}_genericvirtualsensorsandactuators)
