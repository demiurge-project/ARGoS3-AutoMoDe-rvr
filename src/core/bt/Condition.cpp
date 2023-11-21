/*
 * @file <src/core/bt/Condition.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe-BT
 *
 * @license MIT License
 */

#include "Condition.h"

namespace argos {

  /****************************************/
  /****************************************/

	Condition::Condition() {
    m_strLabel = "Condition";
		m_strBranchId = "";
  }

  /****************************************/
	/****************************************/

  Condition::~Condition() {
		delete m_pcCondition;
	}

  /****************************************/
  /****************************************/


	Condition::Condition(Condition* pc_action) {
		m_pcCondition = pc_action->GetCondition()->Clone();
	}

	/****************************************/
	/****************************************/

	Node::ReturnState Condition::Tick() {
		//LOG << m_strLabel << m_strBranchId  << " (" << m_pcCondition->GetLabel() << ")" << std::endl;
		Node::ReturnState eCurrentState;
		if (m_pcCondition->Verify()) {
			//LOG << "---> Success" << std::endl;
			eCurrentState = Node::SUCCESS;
		} else {
			//LOG << "---> Failure" << std::endl;
			eCurrentState = Node::FAILURE;
		}
		return eCurrentState;
	}

	/****************************************/
	/****************************************/

	void Condition::Reset() {}

	/****************************************/
	/****************************************/

	Condition* Condition::Clone() {
		return new Condition(this);
	}

  /****************************************/
  /****************************************/

  void Condition::FillDOTDescription(std::ostringstream& ss_dot_string){
		// Creation of graphical nodes
		ss_dot_string << m_strLabel << m_strBranchId << " [shape=diamond;label=\"" << m_pcCondition->GetDOTDescription() << "\";color=green];";
  }

	/****************************************/
	/****************************************/

	std::string Condition::GetLabel() {
	 return m_strLabel;
	}

	/****************************************/
	/****************************************/

	void Condition::AddChildNode(Node* pc_new_child_node) {
		THROW_ARGOSEXCEPTION("An Condition node should not have children");
	}

	/****************************************/
	/****************************************/

	void Condition::ShareRobotDAO(RVRDAO* pc_robot_dao) {
		m_pcCondition->SetRobotDAO(pc_robot_dao);
	}

	/****************************************/
	/****************************************/

	void Condition::SetCondition(AutoMoDeCondition* pc_condition) {
		m_pcCondition = pc_condition;
	}

	/****************************************/
	/****************************************/

	AutoMoDeCondition* Condition::GetCondition() {
		return m_pcCondition;
	}
}
