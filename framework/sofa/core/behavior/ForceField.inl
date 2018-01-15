/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 INRIA, USTL, UJF, CNRS, MGH                    *
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
#ifndef SOFA_CORE_BEHAVIOR_FORCEFIELD_INL
#define SOFA_CORE_BEHAVIOR_FORCEFIELD_INL

#include <sofa/core/behavior/ForceField.h>
#ifdef SOFA_SMP
#include <sofa/defaulttype/SharedTypes.h>
#endif
#include <iostream>
#include <numeric>

namespace sofa
{

namespace core
{

namespace behavior
{


template<class DataTypes>
ForceField<DataTypes>::ForceField(MechanicalState<DataTypes> *mm)
    : BaseForceField()
    , mstate(initLink("mstate", "MechanicalState used by this ForceField"), mm)
    , d_drawStatsArrowWidth(initData(&d_drawStatsArrowWidth, 0.03f, "drawStatsArrowWidth", "Arrow width"))
    , d_drawStatsArrowScaleLength(initData(&d_drawStatsArrowScaleLength, 0.0008f, "drawStatsArrowScaleLength", "Arrow length"))
    , d_drawStatsSpheresRadius(initData(&d_drawStatsSpheresRadius, 0.03f, "drawStatsSpheresRadius", "Scale for spheres radius"))
    , d_computeStatsOnAddForce(initData(&d_computeStatsOnAddForce, false, "computeStatsOnAddForce", "Compute statistics if implemented (beware, might be costy !)"))
    , d_statsNumberOfActiveDofs(initData(&d_statsNumberOfActiveDofs, 0u, "statsNumberOfActiveDofs", "Number of dofs that the forcefield applies upon"))
    , d_statsMaxAddForce(initData(&d_statsMaxAddForce, Real(0.0), "statsMaxAddForce", "Maximum norm of the force value applied by the forcefield"))
    , d_statsMeanAddForce(initData(&d_statsMeanAddForce, Real(0.0), "statsMeanAddForce", "Mean norm of the force value applied by the forcefield"))
    , d_statsMedianAddForce(initData(&d_statsMedianAddForce, Real(0.0), "statsMedianAddForce", "Median norm of the force value applied by the forcefield"))
    , d_statsMinAddForce(initData(&d_statsMinAddForce, Real(0.0), "statsMinAddForce", "Minimum norm of the force value applied by the forcefield"))
    , d_drawStatsForces(initData(&d_drawStatsForces, false, "drawStatsForces", "Activate debug draw of the computed addForce increment"))
    , d_drawStatsForcesColor(initData(&d_drawStatsForcesColor, "drawStatsForcesColor", "Color for the debug draw of the computed addForce increment"))
    , d_drawStatsActiveDofs(initData(&d_drawStatsActiveDofs, false, "drawStatsActiveDofs", "Draw the mechanical dofs on which the forcefield is applied"))
    , d_drawStatsActiveDofsColor(initData(&d_drawStatsActiveDofsColor, "drawStatsActiveDofsColor", "Color for the debug draw of the mechanical dofs on which the forcefield is applied"))
    , d_statsAddForces(initData(&d_statsAddForces, "statsAddForces", "Vector of the increment of force computed by the addForce() method"))
{
    d_computeStatsOnAddForce.setGroup("Stats_");
    d_statsNumberOfActiveDofs.setGroup("Stats_");
    d_statsMaxAddForce.setGroup("Stats_");
    d_statsMeanAddForce.setGroup("Stats_");
    d_statsMedianAddForce.setGroup("Stats_");
    d_statsMinAddForce.setGroup("Stats_");

    d_statsNumberOfActiveDofs.setReadOnly(true);
    d_statsMaxAddForce.setReadOnly(true);
    d_statsMeanAddForce.setReadOnly(true);
    d_statsMedianAddForce.setReadOnly(true);
    d_statsMinAddForce.setReadOnly(true);

    d_drawStatsForcesColor.setGroup("Visualization Stats Forces_");
    d_drawStatsForces.setGroup("Visualization Stats Forces_");
    d_drawStatsActiveDofs.setGroup("Visualization Stats Forces_");
    d_drawStatsActiveDofsColor.setGroup("Visualization Stats Forces_");

    d_drawStatsArrowScaleLength.setGroup("Visualization Stats Params_");
    d_drawStatsArrowWidth.setGroup("Visualization Stats Params_");
    d_drawStatsSpheresRadius.setGroup("Visualization Stats Params_");

    sofa::helper::OptionsGroup* drawForcesColor = d_drawStatsForcesColor.beginEdit();
    drawForcesColor->setNames(core::visual::DrawTool::GetDefaultColorNames());
    drawForcesColor->setSelectedItem("white");
    d_drawStatsForcesColor.endEdit();

    sofa::helper::OptionsGroup* drawActiveDofsColor = d_drawStatsActiveDofsColor.beginEdit();
    drawActiveDofsColor->setNames(core::visual::DrawTool::GetDefaultColorNames());
    drawActiveDofsColor->setSelectedItem("orange");
    d_drawStatsActiveDofsColor.endEdit();
}

template<class DataTypes>
ForceField<DataTypes>::~ForceField()
{
}

template<class DataTypes>
void ForceField<DataTypes>::init()
{
    BaseForceField::init();

    if (!mstate.get())
    {
        mstate.set(MechanicalState<DataTypes>::DynamicCast(getContext()->getMechanicalState()));
    }
    m_statsTopology = this->getContext()->getActiveMeshTopology();
}

#ifdef SOFA_SMP
template<class DataTypes>
struct ParallelForceFieldAddForce
{
    void operator()(const MechanicalParams *mparams /* PARAMS FIRST */, ForceField< DataTypes > *ff,Shared_rw< objectmodel::Data< typename DataTypes::VecDeriv > > _f,Shared_r< objectmodel::Data< typename DataTypes::VecCoord > > _x,Shared_r< objectmodel::Data< typename DataTypes::VecDeriv> > _v)
    {
        ff->addForce(mparams /* PARAMS FIRST */, _f.access(),_x.read(),_v.read());
    }
};

template<class DataTypes>
struct ParallelForceFieldAddDForce
{
    void operator()(const MechanicalParams *mparams /* PARAMS FIRST */, ForceField< DataTypes >*ff,Shared_rw< objectmodel::Data< typename DataTypes::VecDeriv> > _df,Shared_r<objectmodel::Data< typename DataTypes::VecDeriv> > _dx)
    {
        ff->addDForce(mparams /* PARAMS FIRST */, _df.access(),_dx.read());
    }
};
#endif /* SOFA_SMP */


template<class DataTypes>
void ForceField<DataTypes>::addForce(const MechanicalParams* mparams /* PARAMS FIRST */, MultiVecDerivId fId )
{
    if (mparams)
    {
        const bool storeStatsAddForces = this->storeStatsAddForces();
        if (!storeStatsAddForces)
        {
            addForce(mparams /* PARAMS FIRST */, *fId[mstate.get(mparams)].write(), *mparams->readX(mstate), *mparams->readV(mstate));
        }
        else
        {
            sofa::helper::WriteAccessor< DataVecDeriv > addForces = d_statsAddForces;
            addForces.clear();
            addForce(mparams /* PARAMS FIRST */, d_statsAddForces, *mparams->readX(mstate), *mparams->readV(mstate));

            if (!addForces.empty())
            {
                DataVecDeriv& f = *fId[mstate.get(mparams)].write();
                sofa::helper::WriteAccessor< DataVecDeriv > f_out = f;
                f_out.resize(addForces.size());
                for (std::size_t i = 0; i < addForces.size(); ++i)
                {
                    f_out[i] += addForces[i];
                }

                if (d_computeStatsOnAddForce.getValue())
                {
                    computeStatsOnAddForce();
                }
            }


        }
    }
}

template<class DataTypes>
void ForceField<DataTypes>::addDForce(const MechanicalParams* mparams /* PARAMS FIRST */, MultiVecDerivId dfId )
{
    if (mparams)
    {
#ifndef NDEBUG
        mparams->setKFactorUsed(false);
#endif
        addDForce(mparams /* PARAMS FIRST */, *dfId[mstate.get(mparams)].write(), *mparams->readDx(mstate.get(mparams)));

#ifndef NDEBUG
        if (!mparams->getKFactorUsed())
            serr << "WARNING " << getClassName() << " (in ForceField<DataTypes>::addDForce): please use mparams->kFactor() in addDForce" << sendl;
#endif
    }
}

template<class DataTypes>
double ForceField<DataTypes>::getPotentialEnergy(const MechanicalParams* mparams) const
{

    if (this->mstate)
        return getPotentialEnergy(mparams /* PARAMS FIRST */, *mparams->readX(mstate));
    return 0;
}

template<class DataTypes>
void ForceField<DataTypes>::addKToMatrix(const MechanicalParams* mparams /* PARAMS FIRST */, const sofa::core::behavior::MultiMatrixAccessor* matrix )
{
    sofa::core::behavior::MultiMatrixAccessor::MatrixRef r = matrix->getMatrix(this->mstate);
    if (r) 
    {
        addKToMatrix(r.matrix, mparams->kFactor(), r.offset);
    }
    else 
    {
        serr << "ERROR(" << getClassName() << "): addKToMatrix found no valid matrix accessor." << sendl;
    }
}

template<class DataTypes>
void ForceField<DataTypes>::addKToMatrix(sofa::defaulttype::BaseMatrix * /*mat*/, double /*kFact*/, unsigned int &/*offset*/)
{
    serr << "ERROR("<<getClassName()<<"): addKToMatrix not implemented." << sendl;
}

template<class DataTypes>
void ForceField<DataTypes>::addSubKToMatrix(const MechanicalParams* mparams /* PARAMS FIRST */, const sofa::core::behavior::MultiMatrixAccessor* matrix, const helper::vector<unsigned> & subMatrixIndex)
{
    sofa::core::behavior::MultiMatrixAccessor::MatrixRef r = matrix->getMatrix(this->mstate);
    if (r)
    {
        addSubKToMatrix(r.matrix, subMatrixIndex, mparams->kFactor(), r.offset);
    }
    else
    {
        serr << "ERROR(" << getClassName() << "): addKToMatrix found no valid matrix accessor." << sendl;
    }
}

template<class DataTypes>
void ForceField<DataTypes>::addSubKToMatrix(sofa::defaulttype::BaseMatrix * mat, const helper::vector<unsigned> & /*subMatrixIndex*/, double kFact, unsigned int & offset)
{
    addKToMatrix(mat,kFact,offset);
}




template<class DataTypes>
void ForceField<DataTypes>::addBToMatrix(const MechanicalParams* mparams /* PARAMS FIRST */, const sofa::core::behavior::MultiMatrixAccessor* matrix)
{
    sofa::core::behavior::MultiMatrixAccessor::MatrixRef r = matrix->getMatrix(this->mstate);
    if (r)
        addBToMatrix(r.matrix, mparams->bFactor() , r.offset);
}
template<class DataTypes>
void ForceField<DataTypes>::addBToMatrix(sofa::defaulttype::BaseMatrix * /*mat*/, double /*bFact*/, unsigned int &/*offset*/)
{
//    serr << "ERROR("<<getClassName()<<"): addBToMatrix not implemented." << sendl;
}

template<class DataTypes>
void ForceField<DataTypes>::addSubBToMatrix(const MechanicalParams* mparams /* PARAMS FIRST */, const sofa::core::behavior::MultiMatrixAccessor* matrix, const helper::vector<unsigned> & subMatrixIndex)
{
    sofa::core::behavior::MultiMatrixAccessor::MatrixRef r = matrix->getMatrix(this->mstate);
    if (r) addSubBToMatrix(r.matrix, subMatrixIndex, mparams->bFactor() , r.offset);
}

template<class DataTypes>
void ForceField<DataTypes>::addSubBToMatrix(sofa::defaulttype::BaseMatrix * mat, const helper::vector<unsigned> & /*subMatrixIndex*/, double bFact, unsigned int & offset)
{
    addBToMatrix(mat,bFact,offset);
}


template<class DataTypes>
void ForceField<DataTypes>::getStatsActivePoints(helper::vector<core::topology::Topology::PointID>& pointsInTopology)
{
    if (!m_statsTopology || !mstate) return;
    pointsInTopology.clear();
    
    core::topology::TopologyObjectType topologyType = m_statsTopology->getTopologyType();
    switch (topologyType)
    {
    case core::topology::TopologyObjectType::HEXAHEDRON:
    {
        for (int pid = 0; pid < mstate->getSize(); ++pid)
        {
            if (!m_statsTopology->getHexahedraAroundVertex(pid).empty())
            {
                pointsInTopology.push_back(pid);
            }
        }
        break;
    }
    case core::topology::TopologyObjectType::TETRAHEDRON:
    {
        for (int pid = 0; pid < mstate->getSize(); ++pid)
        {
            if (!m_statsTopology->getTetrahedraAroundVertex(pid).empty())
            {
                pointsInTopology.push_back(pid);
            }
        }
        break;
    }
    case core::topology::TopologyObjectType::QUAD:
    {
        for (int pid = 0; pid < mstate->getSize(); ++pid)
        {
            if (!m_statsTopology->getQuadsAroundVertex(pid).empty())
            {
                pointsInTopology.push_back(pid);
            }
        }
        break;
    }
    case core::topology::TopologyObjectType::TRIANGLE:
    {
        for (int pid = 0; pid < mstate->getSize(); ++pid)
        {
            if (!m_statsTopology->getTrianglesAroundVertex(pid).empty())
            {
                pointsInTopology.push_back(pid);
            }
        }
        break;
    }
    case core::topology::TopologyObjectType::EDGE:
    {
        for (int pid = 0; pid < mstate->getSize(); ++pid)
        {
            if (!m_statsTopology->getEdgesAroundVertex(pid).empty())
            {
                pointsInTopology.push_back(pid);
            }
        }
        break;
    }
    default:
    {
        break;
    }
    }
}

template<class DataTypes>
void ForceField<DataTypes>::computeStatsOnAddForce()
{
    sofa::helper::ReadAccessor< DataVecDeriv > statsAddForces = d_statsAddForces;

    d_statsMaxAddForce.setValue(Real(0));
    d_statsMeanAddForce.setValue(Real(0));
    d_statsMedianAddForce.setValue(Real(0));
    d_statsMinAddForce.setValue(Real(0));

    if (statsAddForces.empty()) return;

    using PointID = core::topology::Topology::PointID;

    helper::vector<PointID> pointsInTopology;
    getStatsActivePoints(pointsInTopology);

    // normalize each element of the addForces vector
    helper::vector<Real> addForcesN;
    if (!pointsInTopology.empty())
    {
        addForcesN.reserve(pointsInTopology.size());
        for (PointID pid: pointsInTopology)
        {
            addForcesN.push_back(statsAddForces[pid].norm());
        }
    }
    else
    {
        addForcesN.reserve(statsAddForces.size());
        for (const Deriv& addForce : statsAddForces)
        {
            addForcesN.push_back(addForce.norm());
        }
    }

    std::sort(addForcesN.begin(), addForcesN.end());

    Real median(0);
    if (addForcesN.size() % 2 == 0)
    {
        median = (addForcesN[(addForcesN.size()) / 2 - 1] + addForcesN[(addForcesN.size()) / 2]) / 2;
    }
    else
    {
        median = addForcesN[(addForcesN.size() - 1) / 2];
    }

    const Real max = addForcesN[addForcesN.size() - 1];
    const Real mean = (!addForcesN.empty())? std::accumulate(addForcesN.begin(), addForcesN.end(), Real(0)) / addForcesN.size() : Real(0);
    const Real min = addForcesN[0];

    d_statsMaxAddForce.setValue(max);
    d_statsMeanAddForce.setValue(mean);
    d_statsMedianAddForce.setValue(median);
    d_statsMinAddForce.setValue(min);

    d_statsNumberOfActiveDofs.setValue(addForcesN.size());
}

template<class DataTypes>
void ForceField<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (!mstate || (!d_drawStatsForces.getValue() && !d_drawStatsActiveDofs.getValue())) return;

    using PointID = core::topology::Topology::PointID;

    helper::ReadAccessor<Data<VecCoord> > x = mstate->readPositions();
    sofa::helper::ReadAccessor< DataVecDeriv > statsAddForces = d_statsAddForces;

    if (d_drawStatsForces.getValue() && statsAddForces.size() == x.size())
    {
        float drawStatsArrowWidth = d_drawStatsArrowWidth.getValue();
        float drawStatsArrowScaleLength = d_drawStatsArrowScaleLength.getValue();
        
        const sofa::defaulttype::Vec4f color = vparams->drawTool()->getColor(d_drawStatsForcesColor.getValue().getSelectedItem());

        for (unsigned int i = 0; i < x.size(); ++i)
        {
            vparams->drawTool()->drawArrow(defaulttype::Vec3d(DataTypes::getCPos(x[i])), defaulttype::Vec3d(DataTypes::getCPos(x[i])) + defaulttype::Vec3d(DataTypes::getDPos(statsAddForces[i])) * drawStatsArrowScaleLength, drawStatsArrowWidth, color);
        }
    }
    
    if (d_drawStatsActiveDofs.getValue())
    {
        helper::vector<PointID> pointsInTopology;
        getStatsActivePoints(pointsInTopology);

        sofa::helper::vector<sofa::defaulttype::Vector3> positions;
        float drawStatsSpheresRadius = d_drawStatsSpheresRadius.getValue();

        if (!pointsInTopology.empty())
        {
            for (PointID pid: pointsInTopology)
            {
                positions.push_back(defaulttype::Vec3d(DataTypes::getCPos(x[pid])));
            }
        }
        else
        {
            for (unsigned int i = 0; i < x.size(); ++i)
            {
                positions.push_back(defaulttype::Vec3d(DataTypes::getCPos(x[i])));
            }
        }
        const sofa::defaulttype::Vec4f color = vparams->drawTool()->getColor(d_drawStatsActiveDofsColor.getValue().getSelectedItem());
        vparams->drawTool()->drawSpheres(positions, drawStatsSpheresRadius, color);
    }
}


} // namespace behavior

} // namespace core

} // namespace sofa

#endif
