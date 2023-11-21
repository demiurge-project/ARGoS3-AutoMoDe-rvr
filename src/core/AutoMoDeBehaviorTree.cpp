 /*
  * @file <src/core/AutoMoDeBehaviorTree.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#include "AutoMoDeBehaviorTree.h"

namespace argos {

	/****************************************/
	/****************************************/

	AutoMoDeBehaviorTree::AutoMoDeBehaviorTree() {
		m_unTimeStep = 0;
	}

	/****************************************/
	/****************************************/

	AutoMoDeBehaviorTree::~AutoMoDeBehaviorTree() {
		delete m_pcRootNode;
	}

	/****************************************/
	/****************************************/

	AutoMoDeBehaviorTree::AutoMoDeBehaviorTree(const AutoMoDeBehaviorTree* pc_behavior_tree) {
		Node* pcRootNode = pc_behavior_tree->GetRootNode();
		m_pcRootNode = pcRootNode->Clone();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviorTree::Init() {
		ShareRobotDAO();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviorTree::Reset(){
		m_unTimeStep = 0;
		m_pcRootNode->Reset();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviorTree::ControlStep(){
		m_unTimeStep = m_unTimeStep + 1;

		/* Tick the tree and execute RUNNING leaf */
		m_eTreeStatus = m_pcRootNode->Tick();

		/* If state returned by the tree is SUCCESS, then no action has been performed, and the robot should not move.
		 * However, not all conditions have been tested if the pervious RUNNING branch is not the first (left) branch
		 */
		if ((m_eTreeStatus == Node::SUCCESS) || (m_eTreeStatus == Node::FAILURE)) {
			LOG << "STOP" << std::endl;
			m_pcRobotDAO->SetWheelsVelocity(0.0, 0.0);
		}
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviorTree::SetRootNode(Node* pc_root_node) {
		m_pcRootNode = pc_root_node;
	}

	/****************************************/
	/****************************************/

	Node* AutoMoDeBehaviorTree::GetRootNode() const{
		return m_pcRootNode;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviorTree::SetRobotDAO(RVRDAO* pc_robot_dao) {
		m_pcRobotDAO = pc_robot_dao;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeBehaviorTree::ShareRobotDAO() {
		m_pcRootNode->ShareRobotDAO(m_pcRobotDAO);
	}

	/****************************************/
	/****************************************/

	const UInt32& AutoMoDeBehaviorTree::GetTimeStep() const {
		return m_unTimeStep;
	}

	/****************************************/
	/****************************************/

	const std::string AutoMoDeBehaviorTree::GetReadableFormat() {
		std::ostringstream ssUrl;
		ssUrl << "http://chart.googleapis.com/chart?cht=gv:dot&chl=digraph G{" ;
		m_pcRootNode->FillDOTDescription(ssUrl);
		ssUrl << "}" ;
		std::string strUrl = ssUrl.str();
		std::replace(strUrl.begin(), strUrl.end(), ' ', '+');
		return strUrl;
	}
}
