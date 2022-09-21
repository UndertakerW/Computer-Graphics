#include "tinystr.h"  
#include "tinyxml.h"
#include "xml.h"
#include <eigen3/Eigen/Eigen>
#include <iostream>


using namespace std;

bool AssignXyz(Eigen::Vector3f &v, TiXmlElement* element)
{
	TiXmlElement* xElement = element->FirstChildElement();
	v[0] = atof(xElement->GetText());
	TiXmlElement* yElement = xElement->NextSiblingElement();
	v[1] = atof(yElement->GetText());
	TiXmlElement* zElement = yElement->NextSiblingElement();
	v[2] = atof(zElement->GetText());
	return true;
}

bool ReadParaXml(string m_strXmlPath, Eigen::Vector3f &eye_pos, float &angle,
	Eigen::Vector3f &T, Eigen::Vector3f &S, Eigen::Vector3f &P0, Eigen::Vector3f &P1,
	std::vector<Eigen::Vector3f> &pos, float &eye_fov, float &aspect_ratio, float &zNear, float &zFar)
{

	// Load values from xml file
	TiXmlDocument* Document = new TiXmlDocument();
	if (!Document->LoadFile(m_strXmlPath.c_str()))
	{
		clog << "Failed to load xml file: " << m_strXmlPath << endl;
		cin.get();
		return false;
	}

	// root element
	TiXmlElement* rootElement = Document->RootElement();

	// next element
	TiXmlElement* currentElement = rootElement->FirstChildElement();

	while (currentElement != NULL)
	{
		if (currentElement->ValueTStr() == "eye_position")
		{
			AssignXyz(eye_pos, currentElement);
		}

		else if (currentElement->ValueTStr() == "rotation_angle")
		{
			angle = atof(currentElement->GetText());
		}

		else if (currentElement->ValueTStr() == "T")
		{
			AssignXyz(T, currentElement);
		}

		else if (currentElement->ValueTStr() == "S")
		{
			AssignXyz(S, currentElement);
		}

		else if (currentElement->ValueTStr() == "P0")
		{
			AssignXyz(P0, currentElement);
		}

		else if (currentElement->ValueTStr() == "P1")
		{
			AssignXyz(P1, currentElement);
		}

		else if (currentElement->ValueTStr() == "pos")
		{
			TiXmlElement* childElement = currentElement->FirstChildElement();

			while (childElement != NULL)
			{
				if (childElement->ValueTStr() == "pos0") 
				{
					AssignXyz(pos[0], childElement);
				}

				else if (childElement->ValueTStr() == "pos1")
				{
					AssignXyz(pos[1], childElement);
				}

				else if (childElement->ValueTStr() == "pos2")
				{
					AssignXyz(pos[2], childElement);
				}

				childElement = childElement->NextSiblingElement();
			}
		}

		else if (currentElement->ValueTStr() == "eye_fov")
		{
			eye_fov = atof(currentElement->GetText());
		}

		else if (currentElement->ValueTStr() == "aspect_ratio")
		{
			aspect_ratio = atof(currentElement->GetText());
		}

		else if (currentElement->ValueTStr() == "zNear")
		{
			zNear = atof(currentElement->GetText());
		}

		else if (currentElement->ValueTStr() == "zFar")
		{
			zFar = atof(currentElement->GetText());
		}

		currentElement = currentElement->NextSiblingElement();
	}

	delete Document;
	clog << "Finish xml parsing" << endl;
	return true;
}
