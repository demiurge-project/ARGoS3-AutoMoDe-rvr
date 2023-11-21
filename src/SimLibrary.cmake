# Set up ARGoS compilation information
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})

set(BEHAVIOR_TREE_HEADERS
	core/AutoMoDeBehaviorTree.h
	core/AutoMoDeBehaviorTreeBuilder.h
	core/bt/Action.h
	core/bt/Condition.h
	core/bt/NegationDecorator.h
	core/bt/Node.h
	core/bt/Selector.h
	core/bt/SelectorStar.h
	core/bt/Sequence.h
	core/bt/SequenceStar.h
	#AutoMoDe
	core/AutoMoDeController.h
	# Behaviours
	modules/AutoMoDeBehaviour.h
	modules/AutoMoDeBehaviourAntiPhototaxis.h
	modules/AutoMoDeBehaviourAttraction.h
	modules/AutoMoDeBehaviourExploration.h
	modules/AutoMoDeBehaviourCurve.h
	modules/AutoMoDeBehaviourPhototaxis.h
	modules/AutoMoDeBehaviourRepulsion.h
	modules/AutoMoDeBehaviourRotation.h
	modules/AutoMoDeBehaviourStop.h
	modules/AutoMoDeBehaviourStraight.h
	# Conditions
	modules/AutoMoDeCondition.h
	modules/AutoMoDeConditionBlackFloor.h
	modules/AutoMoDeConditionCloseToNeighbors.h
	modules/AutoMoDeConditionWhiteFloor.h
	modules/AutoMoDeConditionGrayFloor.h
	modules/AutoMoDeConditionNeighborsCount.h
	modules/AutoMoDeConditionInvertedNeighborsCount.h
	modules/AutoMoDeConditionLight.h
	modules/AutoMoDeConditionObstacleInFront.h
	modules/AutoMoDeConditionFixedProbability.h)


set(BEHAVIOR_TREE_SOURCES
	core/AutoMoDeBehaviorTree.cpp
	core/AutoMoDeBehaviorTreeBuilder.cpp
	core/bt/Action.cpp
	core/bt/Condition.cpp
	core/bt/NegationDecorator.cpp
	core/bt/Node.cpp
	core/bt/Selector.cpp
	core/bt/SelectorStar.cpp
	core/bt/Sequence.cpp
	core/bt/SequenceStar.cpp
	#AutoMoDe
	core/AutoMoDeController.cpp
	# Behaviours
	modules/AutoMoDeBehaviour.cpp
	modules/AutoMoDeBehaviourAntiPhototaxis.cpp
	modules/AutoMoDeBehaviourAttraction.cpp
	modules/AutoMoDeBehaviourExploration.cpp
	modules/AutoMoDeBehaviourCurve.cpp
	modules/AutoMoDeBehaviourPhototaxis.cpp
	modules/AutoMoDeBehaviourRepulsion.cpp
	modules/AutoMoDeBehaviourRotation.cpp
	modules/AutoMoDeBehaviourStop.cpp
	modules/AutoMoDeBehaviourStraight.cpp
	# Conditions
	modules/AutoMoDeCondition.cpp
	modules/AutoMoDeConditionBlackFloor.cpp
	modules/AutoMoDeConditionCloseToNeighbors.cpp
	modules/AutoMoDeConditionWhiteFloor.cpp
	modules/AutoMoDeConditionGrayFloor.cpp
	modules/AutoMoDeConditionNeighborsCount.cpp
	modules/AutoMoDeConditionInvertedNeighborsCount.cpp
	modules/AutoMoDeConditionLight.cpp
	modules/AutoMoDeConditionObstacleInFront.cpp
	modules/AutoMoDeConditionFixedProbability.cpp)



add_library(automode_bt SHARED ${BEHAVIOR_TREE_HEADERS} ${BEHAVIOR_TREE_SOURCES})
target_link_libraries(automode_bt argos3plugin_${ARGOS_BUILD_FOR}_rvr)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
add_executable(automode_main_bt AutoMoDeMainBT.cpp)
target_link_libraries(automode_main_bt automode_bt argos3core_${ARGOS_BUILD_FOR} argos3plugin_${ARGOS_BUILD_FOR}_rvr argos3_demiurge_rvr_loop_functions argos3_demiurge_rvr_dao)

add_executable(visualize_bt AutoMoDeVisualizeBT.cpp)
target_link_libraries(visualize_bt automode_bt argos3core_${ARGOS_BUILD_FOR} argos3plugin_${ARGOS_BUILD_FOR}_rvr argos3_demiurge_rvr_dao)
