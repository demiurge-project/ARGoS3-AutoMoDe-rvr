/*
 * @file <src/core/bt/NegationDecorator.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe-BT
 *
 * @license MIT License
 */

#include "NegationDecorator.h"

namespace argos {

  /****************************************/
  /****************************************/

	NegationDecorator::NegationDecorator() {
    m_strLabel = "Negation";
		m_strDOTLabel = "     ~     ";
		m_strBranchId = "";
  }

  /****************************************/
	/****************************************/

  NegationDecorator::~NegationDecorator() {
		for (UInt8 i = 0; i < m_vecChilds.size(); i++) {
			delete m_vecChilds.at(i);
		}
	}

  /****************************************/
  /****************************************/


	NegationDecorator::NegationDecorator(NegationDecorator* pc_selector) {
		std::vector<Node*> vecChildNodes = pc_selector->GetChildNodes();
		m_vecChilds.clear();
		for (std::vector<Node*>::iterator it = vecChildNodes.begin(); it != vecChildNodes.end(); ++it) {
			m_vecChilds.push_back((*it)->Clone());
		}
	}

	/****************************************/
	/****************************************/

	Node::ReturnState NegationDecorator::Tick() {
		//LOG << m_strLabel << m_strBranchId << std::endl;
		Node::ReturnState eCurrentState = m_vecChilds.at(0)->Tick();
		if (eCurrentState == Node::FAILURE) {
			return Node::SUCCESS;
		} else if (eCurrentState == Node::SUCCESS) {
      return Node::FAILURE;
    }
		return eCurrentState;
	}

	/****************************************/
	/****************************************/

	void NegationDecorator::Reset() {}

	/****************************************/
	/****************************************/

	NegationDecorator* NegationDecorator::Clone() {
		return new NegationDecorator(this);
	}

  /****************************************/
  /****************************************/

  void NegationDecorator::FillDOTDescription(std::ostringstream& ss_dot_string){
		ss_dot_string << "node [shape=square;];";
		ss_dot_string << m_strLabel << m_strBranchId << " [label=\"" << m_strDOTLabel << "\"];";
		for (UInt8 i = 0; i < m_vecChilds.size(); i++) {
			ss_dot_string << m_vecChilds.at(i)->GetLabel() << m_vecChilds.at(i)->GetBranchId() << " [label=\"" << m_vecChilds.at(i)->GetDOTLabel() << "\"];";
		}

		// Linking nodes
		for (UInt8 i = 0; i < m_vecChilds.size(); i++) {
			ss_dot_string << m_strLabel << m_strBranchId << " -> " << m_vecChilds.at(i)->GetLabel() << m_vecChilds.at(i)->GetBranchId() << ";";
		}

		// Ask childs to fill DOT description
		for (UInt8 i = 0; i < m_vecChilds.size(); i++) {
			 m_vecChilds.at(i)->FillDOTDescription(ss_dot_string);
		}
  }

	/****************************************/
	/****************************************/

	std::string NegationDecorator::GetLabel() {
	 return m_strLabel;
	}

	/****************************************/
	/****************************************/

	void NegationDecorator::AddChildNode(Node* pc_new_child_node) {
		if (m_vecChilds.size() > 0) {
			THROW_ARGOSEXCEPTION("[FATAL] Attempting of setting more than one child node to Decorator")
		} else {
			m_vecChilds.push_back(pc_new_child_node);
		}
	}

	/****************************************/
	/****************************************/

	void NegationDecorator::ShareRobotDAO(RVRDAO* pc_robot_dao) {
		for (UInt8 i = 0; i < m_vecChilds.size(); i++) {
			m_vecChilds.at(i)->ShareRobotDAO(pc_robot_dao);
		}
	}
}
