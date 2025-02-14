#include "SkeletalModel.h"

#include <FL/Fl.H>
#include <fstream>  // For file I/O
#include <string>   // For std::string
#include <iostream> // degugging

using namespace std;

void SkeletalModel::load(const char *skeletonFile, const char *meshFile, const char *attachmentsFile)
{
	loadSkeleton(skeletonFile);


	m_mesh.load(meshFile);
	m_mesh.loadAttachments(attachmentsFile, m_joints.size());

	computeBindWorldToJointTransforms();
	updateCurrentJointToWorldTransforms();

	cout << "m_joints.size: " << m_joints.size() << '\n';
	cout << "root transformation:\n";
	m_rootJoint->transform.print();
}

void SkeletalModel::draw(Matrix4f cameraMatrix, bool skeletonVisible)
{
	// draw() gets called whenever a redraw is required
	// (after an update() occurs, when the camera moves, the window is resized, etc)

	m_matrixStack.clear();
	m_matrixStack.push(cameraMatrix);

	if( skeletonVisible )
	{
		drawJoints();

		drawSkeleton();
	}
	else
	{
		// Clear out any weird matrix we may have been using for drawing the bones and revert to the camera matrix.
		glLoadMatrixf(m_matrixStack.top().getElements());

		// Tell the mesh to draw itself.
		m_mesh.draw();
	}
}

void SkeletalModel::loadSkeleton( const char* filename )
{
	// Load the skeleton from file here.

	std::ifstream inputFile(filename);
	if (!inputFile) 
	{
        std::cerr << "Error: File could not be opened [in loadSkeleton()]!" << std::endl;
        return;
    }

	float x, y, z;
	int i;

	// root joint
	{
		inputFile >> x >> y >> z >> i;

		Joint *root = new Joint();
		
		// Translation (relative to global)
		root->transform = Matrix4f(
			1, 0, 0, x,
			0, 1, 0, y,
			0, 0, 1, z,
			0, 0, 0, 1
		);

		m_joints.push_back(root);
		m_rootJoint = root;
	}

	// rest of joints
	while (inputFile >> x >> y >> z >> i)
	{		
		Joint *joint = new Joint();
		
		// Translation (relative to parent)
		joint->transform = Matrix4f(
			1, 0, 0, x,
			0, 1, 0, y,
			0, 0, 1, z,
			0, 0, 0, 1
		);

		// Add to parent
		m_joints[i]->children.push_back(joint);

		// Add to list of joints
		m_joints.push_back(joint);
	}
}

void drawJointsHelper(const Joint* joint, MatrixStack& stack)
{
	// Set up joint frame
	stack.push(joint->transform);
	glLoadMatrixf(stack.top().getElements());

	// Draw joint
	glutSolidSphere(0.025f,12,12);

	// Draw children
	for (const Joint* child : joint->children)
	{
		drawJointsHelper(child, stack);
	}

	// Remove joint frame
	stack.pop();
}

void SkeletalModel::drawJoints( )
{
	// Draw a sphere at each joint. You will need to add a recursive helper function to traverse the joint hierarchy.
	//
	// We recommend using glutSolidSphere( 0.025f, 12, 12 )
	// to draw a sphere of reasonable size.
	//
	// You are *not* permitted to use the OpenGL matrix stack commands
	// (glPushMatrix, glPopMatrix, glMultMatrix).
	// You should use your MatrixStack class
	// and use glLoadMatrix() before your drawing call.

	if (m_rootJoint != nullptr)
	{
		drawJointsHelper(m_rootJoint, m_matrixStack);
	}
}

void drawSkeletonHelper(const Joint* joint, MatrixStack& stack)
{
	stack.push(joint->transform);

	// for each child draw a bone that connects this joint (parent) to it (child)
	for (const Joint* child : joint->children)
	{
		// Drawing the stretched cube is a bit complicated due 
		// to glut only drawing cubes centered at the origin.
		// Thus, the coordinate system needs to be transformed
		// to correctly place the cube/bone. 

		// Offset relative to parent
		const Vector3f childOffset = child->transform.getCol(3).xyz();

		// arbitrary vector for finding x, and y
		const Vector3f rnd(0,0,1);

		const Vector3f z = childOffset.normalized();
		const Vector3f y = Vector3f::cross(z,rnd).normalized();
		const Vector3f x = Vector3f::cross(y,z).normalized();

		// Rotation (R)
		// Rotates basis so that z points in same direction as offset
		stack.push(Matrix4f(
			x.x(), y.x(), z.x(), 0,
			x.y(), y.y(), z.y(), 0,
			x.z(), y.z(), z.z(), 0,
			0,     0,     0,     1
		));

		// Scale (S)
		// Scale basis so that a 1x1x1 cube maps to correct size bone
		stack.push(Matrix4f(
			0.025, 0,     0,                  0,
			0,     0.025, 0,                  0,
			0,     0,     childOffset.abs(),  0,
			0,     0,     0,                  1
		));

		// Translation (T)
		// Translate up z axis so that origin is halfway between parent and child
		stack.push(Matrix4f(
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.5,
			0, 0, 0, 1
		));
		
		// set special frame for drawing bone
		glLoadMatrixf(stack.top().getElements());

		// draw the bone
		glutSolidCube(1.0f);

		// pop special transformations for drawing bone
		stack.pop(); // T
		stack.pop(); // S
		stack.pop(); // R

		drawSkeletonHelper(child, stack);
	}

	stack.pop();
}

void SkeletalModel::drawSkeleton( )
{
	// Draw boxes between the joints. You will need to add a recursive helper function to traverse the joint hierarchy.
	drawSkeletonHelper(m_rootJoint, m_matrixStack);
}

void SkeletalModel::setJointTransform(int jointIndex, float rX, float rY, float rZ)
{
	// Set the rotation part of the joint's transformation matrix based on the passed in Euler angles.
	m_joints[jointIndex]->transform.setSubmatrix3x3(0,0, Matrix3f::rotateX(rX) * Matrix3f::rotateY(rY) * Matrix3f::rotateZ(rZ));
}

void computeBindWorldToJointTransformsHelper(Joint* joint, MatrixStack& stack)
{
	stack.push(joint->transform);

	joint->bindWorldToJointTransform = stack.top().inverse();

	for (Joint* child : joint->children)
	{
		computeBindWorldToJointTransformsHelper(child, stack);
	}

	stack.pop();
}

void SkeletalModel::computeBindWorldToJointTransforms()
{
	// 2.3.1. Implement this method to compute a per-joint transform from
	// world-space to joint space in the BIND POSE.
	//
	// Note that this needs to be computed only once since there is only
	// a single bind pose.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.
	MatrixStack stack;

	if (m_rootJoint != nullptr)
	{
		computeBindWorldToJointTransformsHelper(m_rootJoint, stack);
	}
}

void updateCurrentJointToWorldTransformsHelper(Joint * joint, MatrixStack& stack)
{
	stack.push(joint->transform);

	joint->currentJointToWorldTransform = stack.top();

	for (Joint* child : joint->children)
	{
		updateCurrentJointToWorldTransformsHelper(child, stack);
	}

	// Quick test:
	// If still in initial bind pose, then should print the identity matrix.
	// (joint->currentJointToWorldTransform * joint->bindWorldToJointTransform).print();

	stack.pop();	
}

void SkeletalModel::updateCurrentJointToWorldTransforms()
{
	// 2.3.2. Implement this method to compute a per-joint transform from
	// joint space to world space in the CURRENT POSE.
	//
	// The current pose is defined by the rotations you've applied to the
	// joints and hence needs to be *updated* every time the joint angles change.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.

	// Clear camera matrix
	m_matrixStack.clear();

	cout << "Should only see I matrices printed!\n";
	if (m_rootJoint != nullptr)
	{
		updateCurrentJointToWorldTransformsHelper(m_rootJoint, m_matrixStack);
	}
}

void SkeletalModel::updateMesh()
{
	// 2.3.2. This is the core of SSD.
	// Implement this method to update the vertices of the mesh
	// given the current state of the skeleton.
	// You will need both the bind pose world --> joint transforms.
	// and the current joint --> world transforms.

	const std::vector<Vector3f>& bindVertices = m_mesh.bindVertices;
	std::vector<Vector3f>& currentVertices = m_mesh.currentVertices;

	for (unsigned i = 0; i < bindVertices.size(); i++)
	{
		// Current vertex (v)
		const Vector3f& v = bindVertices[i];
		Vector3f weightedPostionOfVertex(0,0,0);
		// Weights for current vertex
		const vector<float>& weights = m_mesh.attachments[i];

		// for (const Joint* joint : m_joints)
		for (unsigned j = 0; j < m_joints.size(); j++)
		{
			const Joint* joint = m_joints[j];
			// Bind pose world to joint (B)
			const Matrix4f& B = joint->bindWorldToJointTransform;
			// Current pose joint to world (T)
			const Matrix4f& T = joint->currentJointToWorldTransform;

			// += T * B * v
			weightedPostionOfVertex += weights[j] * ((T * B) * Vector4f(v,1)).xyz();
		}

		currentVertices[i] = weightedPostionOfVertex;
	}
}

