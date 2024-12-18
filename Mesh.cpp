#include "Mesh.h"
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

void Mesh::load( const char* filename )
{
	// 2.1.1. load() should populate bindVertices, currentVertices, and faces
	
	std::ifstream inputFile(filename);
	if (!inputFile) 
	{
        std::cerr << "Error: File could not be opened [in Mesh::load()]!" << std::endl;
        return;
    }

	std::string line;
	while (std::getline(inputFile, line))
	{
		std::istringstream iss(line);

		char type;

		if (type == 'v') // vertex
		{
			float x, y, z;
			iss >> x >> y >> z;

			bindVertices.push_back(Vector3f(x,y,z));
		}
		else // face
		{
			unsigned int i, j, k; // one-index
			iss >> i >> j >> k;

			// Make zero-index based 
			faces.push_back(Tuple3u(i - 1, j - 1, k - 1));
		}
	}

	// make a copy of the bind vertices as the current vertices
	currentVertices = bindVertices;
}

void Mesh::draw()
{
	// Since these meshes don't have normals
	// be sure to generate a normal per triangle.
	// Notice that since we have per-triangle normals
	// rather than the analytical normals from
	// assignment 1, the appearance is "faceted".
}

void Mesh::loadAttachments( const char* filename, int numJoints )
{
	// 2.2. Implement this method to load the per-vertex attachment weights
	// this method should update m_mesh.attachments
}
