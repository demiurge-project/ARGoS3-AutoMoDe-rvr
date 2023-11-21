/*
 * @file <src/core/AutoMoDeBehaviorTree.h>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 *
 * @brief
 */

#ifndef AUTOMODE_BEHAVIOR_TREE_H
#define AUTOMODE_BEHAVIOR_TREE_H

#include "bt/Node.h"
#include "bt/Action.h"
#include "bt/Condition.h"
#include "bt/NegationDecorator.h"
#include "bt/Selector.h"
#include "bt/SelectorStar.h"
#include "bt/Sequence.h"
#include "bt/SequenceStar.h"
#include "../modules/AutoMoDeBehaviour.h"
#include "../modules/AutoMoDeBehaviourAttraction.h"
#include "../modules/AutoMoDeBehaviourAntiPhototaxis.h"
#include "../modules/AutoMoDeBehaviourCurve.h"
#include "../modules/AutoMoDeBehaviourExploration.h"
#include "../modules/AutoMoDeBehaviourPhototaxis.h"
#include "../modules/AutoMoDeBehaviourRepulsion.h"
#include "../modules/AutoMoDeBehaviourRotation.h"
#include "../modules/AutoMoDeBehaviourStop.h"
#include "../modules/AutoMoDeBehaviourStraight.h"
#include "../modules/AutoMoDeCondition.h"
#include "../modules/AutoMoDeConditionBlackFloor.h"
#include "../modules/AutoMoDeConditionGrayFloor.h"
#include "../modules/AutoMoDeConditionWhiteFloor.h"
#include "../modules/AutoMoDeConditionFixedProbability.h"
#include "../modules/AutoMoDeConditionNeighborsCount.h"
#include "../modules/AutoMoDeConditionInvertedNeighborsCount.h"
#include "../modules/AutoMoDeConditionCloseToNeighbors.h"
#include "../modules/AutoMoDeConditionLight.h"
#include "../modules/AutoMoDeConditionObstacleInFront.h"

#include <string>
#include <ctime>
#include <algorithm>

namespace argos {
	class AutoMoDeBehaviorTree {

		public:

			/*
			 * Class constructor.
			 */
			AutoMoDeBehaviorTree();

			/*
			 * Class destructor.
			 */
			virtual ~AutoMoDeBehaviorTree();

			/*
			 * Copy constructor.
			 */
			AutoMoDeBehaviorTree(const AutoMoDeBehaviorTree* pc_fsm);

			/*
			 * Initialize the Behavior Tree.
			 */
			void Init();

			/*
			 * Reset the Behavior Tree.
			 */
			void Reset();

			void ControlStep();

			const UInt32& GetTimeStep() const;

			const std::string GetReadableFormat();

			void SetRootNode(Node* pc_root_node);

			Node* GetRootNode() const;

			void ShareRobotDAO();

			void SetRobotDAO(RVRDAO* m_pcRobotDAO);

		private:

			Node* m_pcRootNode;

			/*
			 * Pointer to the object representing the state of the robot.
			 * @see RVRDAO.
			 */
			RVRDAO* m_pcRobotDAO;

			UInt32 m_unTimeStep;

			Node::ReturnState m_eTreeStatus;

	};
}

#endif
