/*
 * @file <src/AutoMoDeMain.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 */

#include <cctype>
#include <iomanip>
#include <sstream>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/entity.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/utility/plugins/dynamic_loading.h>
#include <argos3/core/simulator/argos_command_line_arg_parser.h>

#include "./core/AutoMoDeBehaviorTree.h"
#include "./core/AutoMoDeBehaviorTreeBuilder.h"

using namespace argos;

const std::string ExplainParameters() {
	std::string strExplanation = "Missing behavior tree configuration. The possible parameters are: \n\n"
		" --bt-config CONF \t The behavior tree description [MANDATORY]\n"
		" --bt-file CONF \t A file containing behavior trees [MANDATORY]\n";
	return strExplanation;
}

// Function Transforms character '"' into '%22'
std::string EncodeURL(const std::string &value) {
    std::ostringstream escaped;
    escaped.fill('0');
    escaped << std::hex;

    for (std::string::const_iterator i = value.begin(), n = value.end(); i != n; ++i) {
        std::string::value_type c = (*i);

				if (c == '"') {
            escaped << '%' << std::setw(2) << int((unsigned char) c);
            continue;
        }
				escaped << c;
    }

    return escaped.str();
}

/**
 * @brief
 *
 */
int main(int n_argc, char** ppch_argv) {


	std::vector<std::string> vecConfigBT;
	bool bBTControllerFound = false;
	bool bBTFileFound = false;
	std::string strBTFile;

	std::vector<AutoMoDeBehaviorTree*> vecBT;

	try {
		// Cutting off the FSM configuration from the command line

		int nCurrentArgument = 1;
		while(!bBTControllerFound && nCurrentArgument < n_argc) {
			if(strcmp(ppch_argv[nCurrentArgument], "--bt-config") == 0) {
				bBTControllerFound = true;
				nCurrentArgument++;
				while (nCurrentArgument < n_argc) {
					vecConfigBT.push_back(std::string(ppch_argv[nCurrentArgument]));
					nCurrentArgument++;
				}
				// Do not take the FSM configuration into account in the standard command line parsing.
				n_argc = n_argc - vecConfigBT.size() - 1;
			}
			nCurrentArgument++;
		}

		nCurrentArgument = 1;
		while(!bBTFileFound && nCurrentArgument < n_argc) {
			if(strcmp(ppch_argv[nCurrentArgument], "--bt-file") == 0) {
				bBTFileFound = true;
				nCurrentArgument++;
				strBTFile = std::string(ppch_argv[nCurrentArgument]);
      }
		}

		AutoMoDeBehaviorTreeBuilder cBuilder = AutoMoDeBehaviorTreeBuilder();

		if (bBTControllerFound) {
			AutoMoDeBehaviorTree* pcBehaviorTree = cBuilder.BuildBehaviorTree(vecConfigBT);

			std::string strBehaviorTreeURL = pcBehaviorTree->GetReadableFormat();

			std::string strBrowser = "firefox \"";
			strBrowser.append(EncodeURL(strBehaviorTreeURL));
			strBrowser.append("\"");

			system(strBrowser.c_str());
		}

		if (bBTFileFound) {
			std::ifstream file(strBTFile.c_str()); // pass file name as argment
		  std::string linebuffer;

			while (file && getline(file, linebuffer)){
				if (linebuffer.length() == 0)continue;
				AutoMoDeBehaviorTree* pcBehaviorTree = cBuilder.BuildBehaviorTree(linebuffer.c_str());

				std::string strBehaviorTreeURL = pcBehaviorTree->GetReadableFormat();
				//std::cout << strFiniteStateMachineURL << std::endl;

				std::string strBrowser = "firefox \"";
				strBrowser.append(EncodeURL(strBehaviorTreeURL));
				strBrowser.append("\"");

				system(strBrowser.c_str());
			}
		}
	} catch(std::exception& ex) {
    // A fatal error occurred: dispose of data, print error and exit
    LOGERR << ex.what() << std::endl;
    return 1;
  }

	/* Everything's ok, exit */
  return 0;
}
