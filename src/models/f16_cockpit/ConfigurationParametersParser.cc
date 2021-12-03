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

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include "ConfigurationParametersParser.hh"

#define SMARTIES_TAG "SMARTIES_CONFIG_ITEM"
#define DEBUG false

using namespace tinyxml2;
using std::cout;
using std::endl;

namespace ConfigurationParser
{

//----------------------------------------------------------------------
//
XMLDocument* loadFileToDoc(string file_url){
    XMLDocument* xml_parameters = new XMLDocument;
    if ( xml_parameters->LoadFile(file_url.c_str()) != 0 ) {
        std::cerr << "Loading of \"" << file_url;
        std::cerr << "\" failed." << endl;
        std::cerr << "Check the location of the file." << endl;
        throw string("File : \"" + file_url + "\" not found ");
    }
    return xml_parameters;
}

//----------------------------------------------------------------------
//
XMLNode* getNextSibling(XMLNode* initialNode, string name, bool inclusive)
{

	XMLNode* node = initialNode;

	// if initial node is not included, skip it
	if(!inclusive)
		node = node->NextSibling();

	// find the node name
	const char* value = node->Value();

	// while the node name is not the given one
	while(strcmp(value, name.c_str())) {

		// find next sibling
		node = node->NextSibling();

		// if there is no other sibling, return NULL
		if(node==NULL)
			return NULL;

		// else update node name
		value = node->Value();
	}
	// return the found node
	return node;
}

//----------------------------------------------------------------------
//
XMLNode* getFirstChild(XMLNode* node, string name)
{
	// find the potential first child
	if(node == NULL)
		return NULL;	
	XMLNode* child = node->FirstChild();
	// if the node has no child, return NULL
	if(child==NULL)
		return NULL;
	// find the first sibling (including that child) having the given name
    return getNextSibling(child, name.c_str(), true);
}

//----------------------------------------------------------------------
//
XMLNode* getFirstConfiguration(XMLNode* root){

	// nodes to be browsed 
	XMLNode* first_item;

	// find federation node (can be root)
    first_item = getNextSibling(root, (char*) SMARTIES_TAG, true);
    if (DEBUG)
	    printf("Configuration item found: %s\n", first_item->Value()); //federation

	return first_item;
}

//----------------------------------------------------------------------
//
const char* getTextValue(XMLDocument* doc, vector<string> path){
  XMLNode* root = doc->FirstChild();
    XMLNode* current_node = getFirstConfiguration(root);
    for(unsigned long i = 0 ; i < path.size(); i++){
        current_node = getFirstChild(current_node, path[i]);
	}
    XMLElement *param = current_node->ToElement();
    if (param->FirstChildElement("value")){
        param = param->FirstChildElement("value");
    }
    return param->GetText();
}

//----------------------------------------------------------------------
//
int getIntValue(XMLDocument* doc, vector<string> path)
{
	return atoi(getTextValue(doc, path));
}

//----------------------------------------------------------------------
//
double getDoubleValue(XMLDocument* doc, vector<string> path)
{
	return atof(getTextValue(doc, path));
}

}
