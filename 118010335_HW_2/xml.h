#include "tinystr.h"  
#include "tinyxml.h"
#include <string>
#include <eigen3/Eigen/Eigen>

bool ReadParaXml(std::string m_strXmlPath, Eigen::Vector3f& eye_pos, float& angle,
	Eigen::Vector3f& T, Eigen::Vector3f& S, Eigen::Vector3f& P0, Eigen::Vector3f& P1,
	std::vector<Eigen::Vector3f>& pos, float& eye_fov, float& aspect_ratio, float& zNear, float& zFar);
