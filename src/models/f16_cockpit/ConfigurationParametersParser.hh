//----------------------------------------------------------------------
//----------------------------------------------------------------------
//
// SMARTIES 
// Simulation Modules for Aircraft Real-Time Embedded Systems
// 
// Copyright (C) 2018-2022  ISAE-SUPAERO
//
// Author: Jean-Baptiste Chaudron
// email:  jean-baptiste.chaudron@isae-supaero.fr
//
//----------------------------------------------------------------------
//----------------------------------------------------------------------

#ifndef CONFIGURATION_PARAMETERS_PARSER_HH
#define CONFIGURATION_PARAMETERS_PARSER_HH

#include "tinyxml2.h"
#include <string>
#include <vector>

using namespace tinyxml2;
using std::string;
using std::vector;

namespace ConfigurationParser{
/** @addtogroup XML 
* @{
*/

/**
 * @brief Load a file into a document and return it.
 * @param file_url location of the file to load.
 * @return the loaded document.
 */
XMLDocument* loadFileToDoc(string file_url);

/**
 * @brief Find the next sibling (including the current node or not) having a given name.
 * @param initialNode The root of search.
 * @param name to search for.
 * @param inclusive Includes the initialNode or not
 * @return The next sibling having the given name.
 */
XMLNode* getNextSibling(XMLNode* initialNode, string name, bool inclusive);

/**
 * @brief Find the first child having a given name
 * @param node Parent node.
 * @param name Name of the child.
 * @return The first child having the given name.
 */
XMLNode* getFirstChild(XMLNode* node, string name);

/**
 * @brief Find the first item node in a tree given by its root
 * @param root of the tree to search.
 * @return The first item found.
 */
XMLNode* getFirstConfiguration(XMLNode* root);

/**
 * @brief Returns the value of the parameter, in text format.
 * @param doc The document where to search for the value.
 * @param path Contains the name of each node in descendant order,
 * starting with the name of the federate and finishing with the
 * name of the parameter.
 * @return The value of the parameter, in text format.
 */
const char* getTextValue(XMLDocument* doc, vector<string> path);

/**
 * @brief Returns the value of the parameter, casted to \p int .
 * @param doc The document where to search for the value.
 * @param path Contains the name of each node in descendant order,
 * starting with the name of the federate and finishing with the
 * name of the parameter.
 * @return The value of the parameter casted to \p int .
 * @note To get an enumeration from the parameters file, use this 
 * method with an appropriate cast :
 * @code enumType enumVar = static_cast<enumType>(getIntValue(doc,path)); @endcode
 */
int getIntValue(XMLDocument* doc, vector<string> path);

/**
 * @brief Returns the value of the parameter, casted to \p double .
 * @param doc The document where to search for the value.
 * @param path Contains the name of each node in descendant order,
 * starting with the name of the federate and finishing with the
 * name of the parameter.
 * @return The value of the parameter casted to \p double .
 */
double getDoubleValue(XMLDocument* doc, vector<string> path);

}

#endif
/// @}
