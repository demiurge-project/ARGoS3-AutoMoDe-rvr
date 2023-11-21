/*
 * @file <src/core/AutoMoDeBehaviorTreeBuilder.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 */

#include "AutoMoDeBehaviorTreeBuilder.h"

namespace argos {

	/****************************************/
	/****************************************/

	AutoMoDeBehaviorTreeBuilder::AutoMoDeBehaviorTreeBuilder() {
		m_unMaxTreeLevel = 5;
	}

	/****************************************/
	/****************************************/

	AutoMoDeBehaviorTreeBuilder::~AutoMoDeBehaviorTreeBuilder() {
		delete cBehaviorTree;
	}

	/****************************************/
	/****************************************/

	AutoMoDeBehaviorTree* AutoMoDeBehaviorTreeBuilder::BuildBehaviorTree(const std::string& str_bt_config) {
		std::istringstream iss(str_bt_config);
		std::vector<std::string> tokens;
		copy(std::istream_iterator<std::string>(iss),
			std::istream_iterator<std::string>(),
			std::back_inserter(tokens));
		return BuildBehaviorTree(tokens);
	}

	/****************************************/
	/****************************************/

	AutoMoDeBehaviorTree* AutoMoDeBehaviorTreeBuilder::BuildBehaviorTree(std::vector<std::string>& vec_bt_config) {
		cBehaviorTree = new AutoMoDeBehaviorTree();
		Node* pcRootNode;

		// Get root node type and create root node.
		std::vector<std::string>::iterator it;
		try {
			it = std::find(vec_bt_config.begin(), vec_bt_config.end(), "--nroot");
			UInt8 unRootNodeType = atoi((*(it+1)).c_str());
			pcRootNode = GetNodeFromType(unRootNodeType);
		}
		catch (std::exception e) {
			THROW_ARGOSEXCEPTION("[Error while parsing]: Could not instanciate Root Node");
		}

		try {
			std::vector<std::string> vecRemainingTree(it+2, vec_bt_config.end());
			std::vector<std::vector<std::string> > vecBranches = ExtractBranches(vecRemainingTree);
			for (UInt8 i = 0; i < vecBranches.size(); i++) {
				pcRootNode->AddChildNode(ParseSubTree(vecBranches.at(i)));
			}
		}
		catch (std::exception e) {
			THROW_ARGOSEXCEPTION("[Error while parsing]: Error while parsing Root's childs");
		}

		cBehaviorTree->SetRootNode(pcRootNode);
		std::cout << "Parsing done succesfully!" << std::endl;
		return cBehaviorTree;
	}

	/****************************************/
	/****************************************/

	Node* AutoMoDeBehaviorTreeBuilder::ParseSubTree(std::vector<std::string>& vec_sub_tree) {
		Node* pcNode;
		std::ostringstream oss;
		std::vector<std::string>::iterator it;

		// std::cout << "Parsing subtree of size " << vec_sub_tree.size() << std::endl;
		// for (it=vec_sub_tree.begin(); it!=vec_sub_tree.end(); it++) {
		// 	std::cout << *(it) << " ";
		// }
		// std::cout << std::endl;

		std::string strNodeIdentifier = (*vec_sub_tree.begin()).substr(3, m_unMaxTreeLevel).c_str();
		UInt8 unNodeType = atoi((*(vec_sub_tree.begin()+1)).c_str());

		if (unNodeType < 5) {   // If not an action or condition
			pcNode = GetNodeFromType(unNodeType);
			oss << "--nchild" << strNodeIdentifier;
			it = std::find(vec_sub_tree.begin(), vec_sub_tree.end(), oss.str());
			if (it != vec_sub_tree.end()) {
				std::vector<std::string> vecRemainingTree(it, vec_sub_tree.end());
				std::vector<std::vector<std::string> > vecBranches = ExtractBranches(vecRemainingTree);
				for (UInt8 i = 0; i < vecBranches.size(); i++) {
					pcNode->AddChildNode(ParseSubTree(vecBranches.at(i)));
				}
			} else {
				THROW_ARGOSEXCEPTION("Error while parsing scheduling node");
			}
		} else if (unNodeType == 5) { // If an Action node
			// std::cout << "Action leaf reached" << std::endl;
			oss << "--a" << strNodeIdentifier;
			it = std::find(vec_sub_tree.begin(), vec_sub_tree.end(), oss.str());
			if (it != vec_sub_tree.end()) {
				std::vector<std::string> vecRemainingTree(it, vec_sub_tree.end());
				pcNode = HandleAction(vecRemainingTree);
			} else {
					THROW_ARGOSEXCEPTION("Error while parsing action");
			}
		} else if (unNodeType == 6) { // If a Condition node
			// std::cout << "Condition leaf reached" << std::endl;
			oss << "--c" << strNodeIdentifier;
			it = std::find(vec_sub_tree.begin(), vec_sub_tree.end(), oss.str());
			if (it != vec_sub_tree.end()) {
				std::vector<std::string> vecRemainingTree(it, vec_sub_tree.end());
				pcNode = HandleCondition(vecRemainingTree);
			} else {
				THROW_ARGOSEXCEPTION("Error while parsing action");
			}
		}
		pcNode->SetBranchId(strNodeIdentifier);
		return pcNode;
	}

	/****************************************/
	/****************************************/

	std::vector<std::vector<std::string> > AutoMoDeBehaviorTreeBuilder::ExtractBranches(std::vector<std::string>& vec_sub_tree) {
		std::vector<std::string>::iterator it;
		std::vector<std::vector<std::string> > vecBranches;
		UInt8 unNumberChilds = atoi((*(vec_sub_tree.begin()+1)).c_str());
		std::vector<std::string>::iterator first_child;
		std::vector<std::string>::iterator second_child;
		std::string strBranchIdentifier;
		if ( (*vec_sub_tree.begin()).compare("--nchildroot") == 0 ) {  // First node (aka root) case
			strBranchIdentifier = "";
		} else {
			strBranchIdentifier = (*vec_sub_tree.begin()).substr(8, m_unMaxTreeLevel+1).c_str(); // All other cases
		}

		for (UInt32 i = 0; i < unNumberChilds; i++) {
			std::ostringstream oss;
			oss << "--n" << strBranchIdentifier << i;
			first_child = std::find(vec_sub_tree.begin(), vec_sub_tree.end(), oss.str());
			if (i+1 < unNumberChilds) {
				std::ostringstream oss;
				oss << "--n" << strBranchIdentifier << i+1;
				second_child = std::find(vec_sub_tree.begin(), vec_sub_tree.end(), oss.str());
			} else {
				second_child = vec_sub_tree.end();
			}
			std::vector<std::string> vecChildConfig(first_child, second_child);
			vecBranches.push_back(vecChildConfig);
		}
		return vecBranches;
	}

	/****************************************/
	/****************************************/

	std::vector<std::string> AutoMoDeBehaviorTreeBuilder::TransformIntoSelectorSubtree(std::vector<std::string>& vec_action_config, std::string str_node_identifier) {
		std::vector<std::string> vecSelectorSubtree;
		std::ostringstream oss;
		oss << "--nchild" << str_node_identifier << " 2 --n" << str_node_identifier << "0 5 --af" << str_node_identifier << "0 ";
		oss << *(vec_action_config.begin() + 1) << " ";

		// Checking for parameters
		std::string vecPossibleParameters[] = {"rwm", "att", "rep", "p", "r"};
		UInt8 unNumberPossibleParameters = sizeof(vecPossibleParameters) / sizeof(vecPossibleParameters[0]);
		std::vector<std::string>::iterator it;
		for (UInt8 i = 0; i < unNumberPossibleParameters; i++) {
			std::string strCurrentParameter = vecPossibleParameters[i];
			std::ostringstream oss_param;
			oss_param << "--" << strCurrentParameter << str_node_identifier;
			it = std::find(vec_action_config.begin(), vec_action_config.end(), oss_param.str());
			if (it != vec_action_config.end()) {
				Real fCurrentParameterValue = strtod((*(it+1)).c_str(), NULL);
				oss << oss_param.str().c_str() << "0 " << fCurrentParameterValue << " ";
			}
		}

		// Addition of Rotation action
		oss << "--n" << str_node_identifier << "1 5 --af" << str_node_identifier << "1 7 --p" << str_node_identifier << "1 0.0";
		//std::cout << oss.str().c_str() << std::endl;

		// Transforming stringstream to vector of strings
		std::istringstream iss(oss.str().c_str());
		copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter(vecSelectorSubtree));
		return vecSelectorSubtree;
	}

	/****************************************/
	/****************************************/

	Action* AutoMoDeBehaviorTreeBuilder::HandleAction(std::vector<std::string>& vec_bt_action_config) {
		Action* pcActionNode = new Action();
		AutoMoDeBehaviour* pcNewBehaviour;
		std::vector<std::string>::iterator it;
		// Extraction of the index of the node
		std::string strNodeIndex =  (*vec_bt_action_config.begin()).substr(3, m_unMaxTreeLevel+1).c_str();
		// Extraction of the identifier of the behaviour
		UInt8 unBehaviourIdentifier =  atoi((*(vec_bt_action_config.begin()+1)).c_str());

		// Creation of the Behaviour object
		switch(unBehaviourIdentifier) {
			case 0:
				pcNewBehaviour = new AutoMoDeBehaviourExploration();
				break;
			case 1:
				pcNewBehaviour = new AutoMoDeBehaviourStop();
				break;
			case 2:
				pcNewBehaviour = new AutoMoDeBehaviourPhototaxis();
				break;
			case 3:
				pcNewBehaviour = new AutoMoDeBehaviourAntiPhototaxis();
				break;
			case 4:
				pcNewBehaviour = new AutoMoDeBehaviourAttraction();
				break;
			case 5:
				pcNewBehaviour = new AutoMoDeBehaviourRepulsion();
				break;
			case 6:
				pcNewBehaviour = new AutoMoDeBehaviourStraight();
				break;
			case 7:
				pcNewBehaviour = new AutoMoDeBehaviourRotation();
				break;
			case 8:
				pcNewBehaviour = new AutoMoDeBehaviourCurve();
				break;
		}
		pcNewBehaviour->SetIndex(0);
		pcNewBehaviour->SetIdentifier(unBehaviourIdentifier);

		// Checking for parameters
		std::string vecPossibleParameters[] = {"rwm", "att", "rep", "p", "r"};
		UInt8 unNumberPossibleParameters = sizeof(vecPossibleParameters) / sizeof(vecPossibleParameters[0]);
		for (UInt8 i = 0; i < unNumberPossibleParameters; i++) {
			std::string strCurrentParameter = vecPossibleParameters[i];
			std::ostringstream oss;
			oss << "--" << strCurrentParameter << strNodeIndex;
			it = std::find(vec_bt_action_config.begin(), vec_bt_action_config.end(), oss.str());
			if (it != vec_bt_action_config.end()) {
				Real fCurrentParameterValue = strtod((*(it+1)).c_str(), NULL);
				pcNewBehaviour->AddParameter(strCurrentParameter, fCurrentParameterValue);
			}
		}
		pcNewBehaviour->Init();
		pcActionNode->SetBehaviour(pcNewBehaviour);
		return pcActionNode;
	}

	/****************************************/
	/****************************************/

	Condition* AutoMoDeBehaviorTreeBuilder::HandleCondition(std::vector<std::string>& vec_bt_condition_config){
		Condition* pcConditionNode = new Condition();
		AutoMoDeCondition* pcNewCondition;

		// Extraction of the index of the node
		std::string strNodeIndex =  (*vec_bt_condition_config.begin()).substr(3, m_unMaxTreeLevel).c_str();
		// Extract Condition identifier
		UInt32 unConditionIdentifier = atoi((*(vec_bt_condition_config.begin()+1)).c_str());

		switch(unConditionIdentifier) {
			case 0:
				pcNewCondition = new AutoMoDeConditionBlackFloor();
				break;
			case 1:
				pcNewCondition = new AutoMoDeConditionGrayFloor();
				break;
			case 2:
				pcNewCondition = new AutoMoDeConditionWhiteFloor();
				break;
			case 3:
				pcNewCondition = new AutoMoDeConditionNeighborsCount();
				break;
			case 4:
				pcNewCondition = new AutoMoDeConditionInvertedNeighborsCount();
				break;
			case 5:
				pcNewCondition = new AutoMoDeConditionFixedProbability();
				break;
			case 6:
				pcNewCondition = new AutoMoDeConditionLight();
				break;
			case 7:
				pcNewCondition = new AutoMoDeConditionObstacleInFront();
				break;
			case 8:
				pcNewCondition = new AutoMoDeConditionCloseToNeighbors();
				break;
		}

		pcNewCondition->SetOriginAndExtremity(0, 0);  // No need of origin and extremity in cas of Behavior Trees. Set them to random value.
		pcNewCondition->SetIndex(0); // Same here, no need of index.
		pcNewCondition->SetIdentifier(unConditionIdentifier);


		// Checking for parameters
		std::vector<std::string>::iterator it;
		std::string vecPossibleParameters[] = {"p", "w", "d"};
		UInt8 unNumberPossibleParameters = sizeof(vecPossibleParameters) / sizeof(vecPossibleParameters[0]);
		for (UInt8 i = 0; i < unNumberPossibleParameters; i++) {
			std::string strCurrentParameter = vecPossibleParameters[i];
			std::stringstream ssParamVariable;
			ssParamVariable << "--" << strCurrentParameter << strNodeIndex;
			it = std::find(vec_bt_condition_config.begin(), vec_bt_condition_config.end(), ssParamVariable.str());
			if (it != vec_bt_condition_config.end()) {
				Real fCurrentParameterValue = strtod((*(it+1)).c_str(), NULL);
				pcNewCondition->AddParameter(strCurrentParameter, fCurrentParameterValue);
			}
		}
		pcNewCondition->Init();

		pcConditionNode->SetCondition(pcNewCondition);
		return pcConditionNode;
	}

	/****************************************/
	/****************************************/

	Node* AutoMoDeBehaviorTreeBuilder::GetNodeFromType(UInt8 un_node_type) {
		Node* pcNode;
		switch(un_node_type) {
			case 0: {
				pcNode = new Selector();
				break;
			}
			case 1: {
				pcNode = new SelectorStar();
				break;
			}
			case 2: {
				pcNode = new Sequence();
				break;
			}
			case 3: {
				pcNode = new SequenceStar();
				break;
			}
			case 4: {
				pcNode = new NegationDecorator();
				break;
			}
			case 5: {
				pcNode = new Action();
				break;
			}
			case 6: {
				pcNode = new Condition();
				break;
			}
		}

		return pcNode;
	}
}
