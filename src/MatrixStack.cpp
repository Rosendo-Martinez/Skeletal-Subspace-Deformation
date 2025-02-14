#include "MatrixStack.h"

MatrixStack::MatrixStack()
{
	// Initialize the matrix stack with the identity matrix.
	m_matrices.push_back(Matrix4f::identity());
}

void MatrixStack::clear()
{
	// Revert to just containing the identity matrix.
	m_matrices.clear();
	m_matrices.push_back(Matrix4f::identity());
}

Matrix4f MatrixStack::top()
{
	// Return the top of the stack

	// 'Top' matrix (T)
	Matrix4f T = Matrix4f::identity();

	// Multiple matrices in stack to get 'Top' matrix
	for (const Matrix4f& M : m_matrices)
	{
		T = T * M;
	}

	return T;
}

void MatrixStack::push( const Matrix4f& m )
{
	// Push m onto the stack.
	// Your stack should have OpenGL semantics:
	// the new top should be the old top multiplied by m

	m_matrices.push_back(m);
}

void MatrixStack::pop()
{
	// Remove the top element from the stack

	m_matrices.pop_back();
}
