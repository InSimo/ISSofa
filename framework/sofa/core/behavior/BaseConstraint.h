/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*            (c) 2006-2021 INRIA, USTL, UJF, CNRS, MGH, InSimo                *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                              SOFA :: Framework                              *
*                                                                             *
* Authors: The SOFA Team (see Authors.txt)                                    *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_CORE_BEHAVIOR_BASECONSTRAINT_H
#define SOFA_CORE_BEHAVIOR_BASECONSTRAINT_H

#include <sofa/core/behavior/BaseConstraintSet.h>
#include <sofa/core/behavior/BaseMechanicalState.h>
#include <sofa/core/behavior/MultiMatrixAccessor.h>

#include <sofa/defaulttype/BaseMatrix.h>
#include <sofa/defaulttype/BaseVector.h>

#include <vector>

namespace sofa
{

namespace core
{

namespace behavior
{

/**
 *  \brief Object computing a constraint resolution within a Gauss-Seidel algorithm
 */

class SOFA_CORE_API ConstraintResolution
{
public:
    ConstraintResolution(unsigned int nbLines, double tolerance = 0.0);

    virtual ~ConstraintResolution() {}

    /// The resolution object can do precomputation with the compliance matrix, and give an initial guess.
    virtual void init(int /*line*/, double** /*w*/, double* /*force*/);

    /// The resolution object can provide an initial guess
    virtual void initForce(int /*line*/, double* /*force*/);

    /// Resolution of the constraint for one Gauss-Seidel iteration
    virtual void resolution(int line, double** w, double* d, double* force, double * dFree) = 0;

    /// Called after Gauss-Seidel last iteration, in order to store last computed forces for the initial guess
    virtual void store(int /*line*/, double* /*force*/, bool /*convergence*/) {}

    /// Optional postStabilize method. Concrete implemntations should only solve here the linear part of the 
    /// constraint problem this constraint resolution. For example frictionnal laws should only care about solving
    /// the unilateral part of the problem, and leave the frictional forces to zero.
    virtual void postStabilize(int line, double** w, double* d, double* force, double* dfree);

    /// Optional addLocalCompliance method.
    virtual void addLocalCompliance(int /*line*/, double** /*w*/) {}

    inline void setNbLines(unsigned int nbLines)
    {
        m_nbLines = nbLines;
    }

    inline unsigned int getNbLines() const
    {
        return m_nbLines;
    }

    inline void setTolerance(double tolerance)
    {
        m_tolerance = tolerance;
    }

    inline double getTolerance() const
    {
        return m_tolerance;
    }

private:
    /// Number of dof used by this particular constraint. To be modified in the object's constructor.
    unsigned int m_nbLines;

    /// Custom tolerance, used for the convergence of this particular constraint instead of the global tolerance
    double m_tolerance;
};

/**
 *  \brief Component computing constraints within a simulated body.
 *
 *  This class define the abstract API common to all constraints.
 *  A BaseConstraint computes constraints applied to one or more simulated body
 *  given its current position and velocity.
 *
 *  Constraints can be internal to a given body (attached to one MechanicalState,
 *  see the Constraint class), or link several bodies together (such as contacts,
 *  see the InteractionConstraint class).
 *
 */
class SOFA_CORE_API BaseConstraint : public BaseConstraintSet
{
public:
    SOFA_ABSTRACT_CLASS_EXTERNAL((BaseConstraint), ((BaseConstraintSet)));

protected:
	BaseConstraint() { };
    virtual ~BaseConstraint() { }
	
private:
	BaseConstraint(const BaseConstraint& n) ;
	BaseConstraint& operator=(const BaseConstraint& n) ;
	
public:
    /// Get the ID of the group containing this constraint. This ID is used to specify which constraints are solved by which solver, by specifying in each solver which groups of constraints it should handle.
    int getGroup() const { return group.getValue(); }

    /// Set the ID of the group containing this constraint. This ID is used to specify which constraints are solved by which solver, by specifying in each solver which groups of constraints it should handle.
    void setGroup(int g) { group.setValue(g); }

    typedef long long PersistentID;
    typedef helper::vector<PersistentID> VecPersistentID;
    typedef defaulttype::Vec<3,double> ConstCoord;
    typedef helper::vector<ConstCoord> VecConstCoord;
    typedef defaulttype::Vec<3,double> ConstDeriv;
    typedef helper::vector<ConstDeriv> VecConstDeriv;
    typedef double ConstArea;
    typedef helper::vector<ConstArea> VecConstArea;

    class ConstraintBlockInfo
    {
    public:
        BaseConstraint* parent;
        int const0; ///< index of first constraint
        int nbLines; ///< how many dofs (i.e. lines in the matrix) are used by each constraint
        int nbGroups; ///< how many groups of constraints are active
        bool hasId; ///< true if this constraint has persistent ID information (one per group)
        bool hasPosition; ///< true if this constraint has coordinates information (one per group)
        bool hasDirection; ///< true if this constraint has direction information (one per line)
        bool hasArea; ///< true if this constraint has area information (one per group)
        int offsetId; ///< index of first constraint group info in vector of persistent ids and coordinates
        int offsetPosition; ///< index of first constraint group info in vector of coordinates
        int offsetDirection; ///< index of first constraint info in vector of directions
        int offsetArea; ///< index of first constraint group info in vector of areas
        ConstraintBlockInfo() : parent(NULL), const0(0), nbLines(1), nbGroups(0), hasId(false), hasPosition(false), hasDirection(false), hasArea(false), offsetId(0), offsetPosition(0), offsetDirection(0), offsetArea(0)
        {}
    };
    typedef helper::vector<ConstraintBlockInfo> VecConstraintBlockInfo;

	/// Get information for each constraint: pointer to parent BaseConstraint, unique persistent ID, 3D position
	/// \param cParams defines the state vectors to use for positions and velocities. Also defines the order of the constraint (POS, VEL, ACC) and resolution parameters (smoothness, ...)
	virtual void getConstraintInfo(const ConstraintParams* cParams, VecConstraintBlockInfo& blocks, VecPersistentID& ids, VecConstCoord& positions, VecConstDeriv& directions, VecConstArea& areas)
	{
        SOFA_UNUSED(cParams);
        SOFA_UNUSED(blocks);
        SOFA_UNUSED(ids);
        SOFA_UNUSED(positions);
        SOFA_UNUSED(directions);
        SOFA_UNUSED(areas);

	}

    /// Add optional local compliance of this constraint to the Compliance Matrix
    /// Local compliance is a diagonal matrix, whose value may depend on the actual constraint measure stored in \phi  
    /// \param cParams the constraint params communicated by the solver to this constraint component
    /// \param C the compliance matrix
    /// \param \phi the constraint measure
    virtual void addCToMatrix(const ConstraintParams* cParams, sofa::defaulttype::BaseMatrix* C, const sofa::defaulttype::BaseVector* phi)
    {
        SOFA_UNUSED(cParams);
        SOFA_UNUSED(C);
        SOFA_UNUSED(phi);
    }

	/// Add the corresponding ConstraintResolution using the offset parameter
	/// \param cParams defines the state vectors to use for positions and velocities. Also defines the order of the constraint (POS, VEL, ACC) and resolution parameters (smoothness, ...)
	/// \param resTab is the result vector that contains the contraint resolution algorithms
    virtual void getConstraintResolution(const ConstraintParams* cParams, std::vector<ConstraintResolution*> &resTab, unsigned int &offset) = 0;


    /// Store the constraint lambda at the constraint dofs at the given VecDerivId location. 
    /// res = J^t * lambda. 
    /// J is the sparse matrix containing the constraint jacobian that was used to build the constraint matrix ( see BaseConstraintSet::buildConstraintMatrix ).
    /// \param cParams stores the id of the state vectors used during the constraint solving step. Mostly it helps retrieving the MatrixDerivId where
    ///        the constraint jacobian J is stored.
    /// \param res is the state vector Id where to store the result.
    /// \param lambda is the vector of scalar constraint impulses. The direction are stored in the MatrixDerivId stored in the cParams.
    virtual void storeLambda(const ConstraintParams* cParams, MultiVecDerivId res, const sofa::defaulttype::BaseVector* lambda) = 0;
};

} // namespace behavior

} // namespace core

} // namespace sofa

#endif
