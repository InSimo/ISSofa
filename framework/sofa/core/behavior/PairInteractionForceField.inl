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
#ifndef SOFA_CORE_BEHAVIOR_PAIRINTERACTIONFORCEFIELD_INL
#define SOFA_CORE_BEHAVIOR_PAIRINTERACTIONFORCEFIELD_INL

#include <sofa/core/behavior/PairInteractionForceField.h>
#include <sofa/core/objectmodel/BaseContext.h>
#include <sofa/core/objectmodel/BaseNode.h>
#include <iostream>
#include <numeric>

namespace sofa
{

namespace core
{

namespace behavior
{

template<class DataTypes>
PairInteractionForceField<DataTypes>::PairInteractionForceField(MechanicalState<DataTypes> *mm1, MechanicalState<DataTypes> *mm2)
    : mstate1(initLink("object1", "First object in interaction"), mm1)
    , mstate2(initLink("object2", "Second object in interaction"), mm2)
    , d_drawStatsArrowWidth(initData(&d_drawStatsArrowWidth, 0.03f, "drawStatsArrowWidth", "Arrow width"))
    , d_drawStatsArrowScaleLength(initData(&d_drawStatsArrowScaleLength, 0.0008f, "drawStatsArrowScaleLength", "Arrow length"))
    , d_drawStatsSpheresRadius(initData(&d_drawStatsSpheresRadius, 0.03f, "drawStatsSpheresRadius", "Scale for spheres radius"))
    , d_computeStatsOnAddForce(initData(&d_computeStatsOnAddForce, false, "computeStatsOnAddForce", "Compute statistics if implemented (beware, might be costy !)"))
    , d_statsNumberOfActiveDofsObj1(initData(&d_statsNumberOfActiveDofsObj1, 0u, "statsNumberOfActiveDofsObj1", "Number of dofs that the forcefield applies upon"))
    , d_statsMaxAddForceObj1(initData(&d_statsMaxAddForceObj1, Real(0.0), "statsMaxAddForceObj1", "Maximum norm of the force value applied by the forcefield"))
    , d_statsMeanAddForceObj1(initData(&d_statsMeanAddForceObj1, Real(0.0), "statsMeanAddForceObj1", "Mean norm of the force value applied by the forcefield"))
    , d_statsMedianAddForceObj1(initData(&d_statsMedianAddForceObj1, Real(0.0), "statsMedianAddForceObj1", "Median norm of the force value applied by the forcefield"))
    , d_statsMinAddForceObj1(initData(&d_statsMinAddForceObj1, Real(0.0), "statsMinAddForceObj1", "Minimum norm of the force value applied by the forcefield"))
    , d_drawStatsForcesObj1(initData(&d_drawStatsForcesObj1, false, "drawStatsForcesObj1", "Activate debug draw of the computed addForce increment"))
    , d_drawStatsForcesColorObj1(initData(&d_drawStatsForcesColorObj1, "drawStatsForcesColorObj1", "Color for the debug draw of the computed addForce increment"))
    , d_drawStatsActiveDofsObj1(initData(&d_drawStatsActiveDofsObj1, false, "drawStatsActiveDofsObj1", "Draw the mechanical dofs on which the forcefield is applied"))
    , d_drawStatsActiveDofsColorObj1(initData(&d_drawStatsActiveDofsColorObj1, "drawStatsActiveDofsColorObj1", "Color for the debug draw of the mechanical dofs on which the forcefield is applied"))
    , d_statsAddForcesObj1(initData(&d_statsAddForcesObj1, "statsAddForcesObj1", "Vector of the increment of force computed by the addForce() method"))
    , d_statsNumberOfActiveDofsObj2(initData(&d_statsNumberOfActiveDofsObj2, 0u, "statsNumberOfActiveDofsObj2", "Number of dofs that the forcefield applies upon"))
    , d_statsMaxAddForceObj2(initData(&d_statsMaxAddForceObj2, Real(0.0), "statsMaxAddForceObj2", "Maximum norm of the force value applied by the forcefield"))
    , d_statsMeanAddForceObj2(initData(&d_statsMeanAddForceObj2, Real(0.0), "statsMeanAddForceObj2", "Mean norm of the force value applied by the forcefield"))
    , d_statsMedianAddForceObj2(initData(&d_statsMedianAddForceObj2, Real(0.0), "statsMedianAddForceObj2", "Median norm of the force value applied by the forcefield"))
    , d_statsMinAddForceObj2(initData(&d_statsMinAddForceObj2, Real(0.0), "statsMinAddForceObj2", "Minimum norm of the force value applied by the forcefield"))
    , d_drawStatsForcesObj2(initData(&d_drawStatsForcesObj2, false, "drawStatsForcesObj2", "Activate debug draw of the computed addForce increment"))
    , d_drawStatsForcesColorObj2(initData(&d_drawStatsForcesColorObj2, "drawStatsForcesColorObj2", "Color for the debug draw of the computed addForce increment"))
    , d_drawStatsActiveDofsObj2(initData(&d_drawStatsActiveDofsObj2, false, "drawStatsActiveDofsObj2", "Draw the mechanical dofs on which the forcefield is applied"))
    , d_drawStatsActiveDofsColorObj2(initData(&d_drawStatsActiveDofsColorObj2, "drawStatsActiveDofsColorObj2", "Color for the debug draw of the mechanical dofs on which the forcefield is applied"))
    , d_statsAddForcesObj2(initData(&d_statsAddForcesObj2, "statsAddForcesObj2", "Vector of the increment of force computed by the addForce() method"))
{
    if (!mm1)
        mstate1.setPath("@./"); // default to state of the current node
    if (!mm2)
        mstate2.setPath("@./"); // default to state of the current node

    d_computeStatsOnAddForce.setGroup("Stats_");
    d_statsNumberOfActiveDofsObj1.setGroup("Stats_");
    d_statsMaxAddForceObj1.setGroup("Stats_");
    d_statsMeanAddForceObj1.setGroup("Stats_");
    d_statsMedianAddForceObj1.setGroup("Stats_");
    d_statsMinAddForceObj1.setGroup("Stats_");

    d_statsNumberOfActiveDofsObj1.setReadOnly(true);
    d_statsMaxAddForceObj1.setReadOnly(true);
    d_statsMeanAddForceObj1.setReadOnly(true);
    d_statsMedianAddForceObj1.setReadOnly(true);
    d_statsMinAddForceObj1.setReadOnly(true);

    d_drawStatsForcesColorObj1.setGroup("Visualization Stats Forces_");
    d_drawStatsForcesObj1.setGroup("Visualization Stats Forces_");
    d_drawStatsActiveDofsObj1.setGroup("Visualization Stats Forces_");
    d_drawStatsActiveDofsColorObj1.setGroup("Visualization Stats Forces_");

    sofa::helper::OptionsGroup* drawForcesColorObj1 = d_drawStatsForcesColorObj1.beginEdit();
    drawForcesColorObj1->setNames(core::visual::DrawTool::GetDefaultColorNames());
    drawForcesColorObj1->setSelectedItem("white");
    d_drawStatsForcesColorObj1.endEdit();

    sofa::helper::OptionsGroup* drawActiveDofsColorObj1 = d_drawStatsActiveDofsColorObj1.beginEdit();
    drawActiveDofsColorObj1->setNames(core::visual::DrawTool::GetDefaultColorNames());
    drawActiveDofsColorObj1->setSelectedItem("orange");
    d_drawStatsActiveDofsColorObj1.endEdit();

    d_statsNumberOfActiveDofsObj2.setGroup("Stats_");
    d_statsMaxAddForceObj2.setGroup("Stats_");
    d_statsMeanAddForceObj2.setGroup("Stats_");
    d_statsMedianAddForceObj2.setGroup("Stats_");
    d_statsMinAddForceObj2.setGroup("Stats_");

    d_statsNumberOfActiveDofsObj2.setReadOnly(true);
    d_statsMaxAddForceObj2.setReadOnly(true);
    d_statsMeanAddForceObj2.setReadOnly(true);
    d_statsMedianAddForceObj2.setReadOnly(true);
    d_statsMinAddForceObj2.setReadOnly(true);

    d_drawStatsForcesColorObj2.setGroup("Visualization Stats Forces_");
    d_drawStatsForcesObj2.setGroup("Visualization Stats Forces_");
    d_drawStatsActiveDofsObj2.setGroup("Visualization Stats Forces_");
    d_drawStatsActiveDofsColorObj2.setGroup("Visualization Stats Forces_");

    d_drawStatsArrowScaleLength.setGroup("Visualization Stats Params_");
    d_drawStatsArrowWidth.setGroup("Visualization Stats Params_");
    d_drawStatsSpheresRadius.setGroup("Visualization Stats Params_");

    sofa::helper::OptionsGroup* drawForcesColorObj2 = d_drawStatsForcesColorObj2.beginEdit();
    drawForcesColorObj2->setNames(core::visual::DrawTool::GetDefaultColorNames());
    drawForcesColorObj2->setSelectedItem("sky");
    d_drawStatsForcesColorObj2.endEdit();

    sofa::helper::OptionsGroup* drawActiveDofsColorObj2 = d_drawStatsActiveDofsColorObj2.beginEdit();
    drawActiveDofsColorObj2->setNames(core::visual::DrawTool::GetDefaultColorNames());
    drawActiveDofsColorObj2->setSelectedItem("purple");
    d_drawStatsActiveDofsColorObj2.endEdit();
}

template<class DataTypes>
PairInteractionForceField<DataTypes>::~PairInteractionForceField()
{
}


template<class DataTypes>
void PairInteractionForceField<DataTypes>::init()
{

    BaseInteractionForceField::init();

    if (mstate1.get() == NULL || mstate2.get() == NULL)
    {
        serr<< "Init of PairInteractionForceField " << getContext()->getName() << " failed!" << sendl;
        //getContext()->removeObject(this);
        return;
    }

    m_statsTopology1 = mstate1->getContext()->getActiveMeshTopology();
    m_statsTopology2 = mstate2->getContext()->getActiveMeshTopology();

}

#ifdef SOFA_SMP
template <class DataTypes>
struct ParallelPairInteractionForceFieldAddForce
{
    void	operator()(const MechanicalParams* mparams /* PARAMS FIRST */, PairInteractionForceField<DataTypes> *ff,
            Shared_rw<objectmodel::Data< typename DataTypes::VecDeriv> > _f1,Shared_rw<objectmodel::Data< typename DataTypes::VecDeriv> > _f2,
            Shared_r<objectmodel::Data< typename DataTypes::VecCoord> > _x1,Shared_r<objectmodel::Data< typename DataTypes::VecCoord> > _x2,
            Shared_r<objectmodel::Data< typename DataTypes::VecDeriv> > _v1,Shared_r<objectmodel::Data< typename DataTypes::VecDeriv> > _v2)
    {
        helper::WriteAccessor< objectmodel::Data<typename DataTypes::VecDeriv> > f1= _f1.access();
        helper::WriteAccessor< objectmodel::Data<typename DataTypes::VecDeriv> > f2= _f2.access();
        helper::ReadAccessor< objectmodel::Data<typename DataTypes::VecCoord> > x1= _x1.read();
        helper::ReadAccessor< objectmodel::Data<typename DataTypes::VecCoord> > x2 = _x2.read();
        if(0&&x1.size()!=f1.size())
        {
            f1.resize(x1.size());
        }
        if(0&&x2.size()!=f2.size())
        {
            f2.resize(x2.size());
        }
        ff->addForce(mparams /* PARAMS FIRST */, _f1.access(),_f2.access(),_x1.read(),_x2.read(),_v1.read(),_v2.read());
    }

    void	operator()(const MechanicalParams *mparams /* PARAMS FIRST */, PairInteractionForceField<DataTypes> *ff,
            Shared_rw<objectmodel::Data< typename DataTypes::VecDeriv> > _f1,
            Shared_r<objectmodel::Data< typename DataTypes::VecCoord> > _x1,
            Shared_r<objectmodel::Data< typename DataTypes::VecDeriv> > _v1)
    {
        helper::WriteAccessor< objectmodel::Data< typename DataTypes::VecDeriv > > f1= _f1.access();

        helper::ReadAccessor< objectmodel::Data< typename DataTypes::VecCoord> > x1= _x1.read();
        helper::ReadAccessor< objectmodel::Data< typename DataTypes::VecDeriv> > v1= _v1.read();
        if(0&&x1.size()!=f1.size())
        {
            f1.resize(x1.size());
        }
        ff->addForce(mparams /* PARAMS FIRST */, _f1.access(),_f1.access(),_x1.read(),_x1.read(),_v1.read(),_v1.read());
    }

};


template <class DataTypes>
struct ParallelPairInteractionForceFieldAddDForce
{
    void	operator()(const MechanicalParams* mparams /* PARAMS FIRST */, PairInteractionForceField<DataTypes> *ff,
            Shared_rw<objectmodel::Data< typename DataTypes::VecDeriv> > _df1,Shared_rw<objectmodel::Data< typename DataTypes::VecDeriv> > _df2,
            Shared_r<objectmodel::Data< typename DataTypes::VecDeriv> > _dx1,Shared_r<objectmodel::Data< typename DataTypes::VecDeriv> > _dx2)
    {
        helper::WriteAccessor< objectmodel::Data<typename DataTypes::VecDeriv> > df1= _df1.access();
        helper::WriteAccessor< objectmodel::Data<typename DataTypes::VecDeriv> > df2= _df2.access();
        helper::ReadAccessor< objectmodel::Data<typename DataTypes::VecDeriv> > dx1 = _dx1.read();
        helper::ReadAccessor< objectmodel::Data<typename DataTypes::VecDeriv> > dx2 = _dx2.read();

        if(0&&dx1.size()!=df1.size())
        {
            df1.resize(dx1.size());
        }
        if(0&&dx2.size()!=df2.size())
        {
            df2.resize(dx2.size());
        }
        // mparams->setKFactor(1.0);
        ff->addDForce(mparams /* PARAMS FIRST */, _df1.access(),_df2.access(),_dx1.read(),_dx2.read());
    }

    void	operator()(const MechanicalParams* mparams /* PARAMS FIRST */, PairInteractionForceField<DataTypes> *ff,Shared_rw< objectmodel::Data< typename DataTypes::VecDeriv> > _df1, Shared_r< objectmodel::Data< typename DataTypes::VecDeriv> > _dx1)
    {
        helper::WriteAccessor< objectmodel::Data<typename DataTypes::VecDeriv> > df1= _df1.access();
        helper::ReadAccessor< objectmodel::Data<typename DataTypes::VecDeriv> > dx1= _dx1.read();

        if(0&&dx1.size()!=df1.size())
        {
            df1.resize(dx1.size());
        }
        // mparams->setKFactor(1.0);
        ff->addDForce(mparams /* PARAMS FIRST */, _df1.access(),_df1.access(),_dx1.read(),_dx1.read());
    }

}; // ParallelPairInteractionForceFieldAddDForce
#endif /* SOFA_SMP */

template<class DataTypes>
void PairInteractionForceField<DataTypes>::addForce(const MechanicalParams* mparams /* PARAMS FIRST */, MultiVecDerivId fId )
{
    if (mstate1 && mstate2)
    {
        const bool storeStatsAddForces = this->storeStatsAddForces();
        if (!storeStatsAddForces)
        {
            addForce( mparams /* PARAMS FIRST */, *fId[mstate1.get(mparams)].write()   , *fId[mstate2.get(mparams)].write()   ,
                    *mparams->readX(mstate1), *mparams->readX(mstate2),
                    *mparams->readV(mstate1), *mparams->readV(mstate2) );
        }
        else
        {
            sofa::helper::WriteAccessor< DataVecDeriv > addForcesObj1 = d_statsAddForcesObj1;
            sofa::helper::WriteAccessor< DataVecDeriv > addForcesObj2 = d_statsAddForcesObj2;
            addForcesObj1.clear();
            addForcesObj2.clear();
            addForce(mparams /* PARAMS FIRST */, d_statsAddForcesObj1, d_statsAddForcesObj2,
                *mparams->readX(mstate1), *mparams->readX(mstate2),
                *mparams->readV(mstate1), *mparams->readV(mstate2));

            if (!addForcesObj1.empty())
            {
                DataVecDeriv& f = *fId[mstate1.get(mparams)].write();
                sofa::helper::WriteAccessor< DataVecDeriv > f_out = f;
                f_out.resize(addForcesObj1.size());
                for (std::size_t i = 0; i < addForcesObj1.size(); ++i)
                {
                    f_out[i] += addForcesObj1[i];
                }

                if (d_computeStatsOnAddForce.getValue())
                {
                    computeStatsOnAddForce(1);
                }
            }

            if (!addForcesObj2.empty())
            {
                DataVecDeriv& f = *fId[mstate2.get(mparams)].write();
                sofa::helper::WriteAccessor< DataVecDeriv > f_out = f;
                f_out.resize(addForcesObj2.size());
                for (std::size_t i = 0; i < addForcesObj2.size(); ++i)
                {
                    f_out[i] += addForcesObj2[i];
                }

                if (d_computeStatsOnAddForce.getValue())
                {
                    computeStatsOnAddForce(2);
                }
            }

        }


    }
    else
        serr<<"PairInteractionForceField<DataTypes>::addForce(const MechanicalParams* /*mparams*/ /* PARAMS FIRST */, MultiVecDerivId /*fId*/ ), mstate missing"<<sendl;
}

template<class DataTypes>
void PairInteractionForceField<DataTypes>::addDForce(const MechanicalParams* mparams /* PARAMS FIRST */, MultiVecDerivId dfId )
{
    if (mstate1 && mstate2)
    {
#ifdef SOFA_SMP
        if (mparams->execMode() == ExecParams::EXEC_KAAPI)
        {
            if (mstate1 == mstate2)
                Task<ParallelPairInteractionForceFieldAddDForce< DataTypes > >(mparams /* PARAMS FIRST */, this,
                        **defaulttype::getShared(*dfId[mstate1.get(mparams)].write()),
                        **defaulttype::getShared(*mparams->readDx(mstate1)));
            else
                Task<ParallelPairInteractionForceFieldAddDForce< DataTypes > >(mparams /* PARAMS FIRST */, this,
                        **defaulttype::getShared(*dfId[mstate1.get(mparams)].write()), **defaulttype::getShared(*dfId[mstate2.get(mparams)].write()),
                        **defaulttype::getShared(*mparams->readDx(mstate1)), **defaulttype::getShared(*mparams->readDx(mstate2)));
        }
        else
#endif /* SOFA_SMP */
            addDForce(
                mparams /* PARAMS FIRST */, *dfId[mstate1.get(mparams)].write()    , *dfId[mstate2.get(mparams)].write()   ,
                *mparams->readDx(mstate1) , *mparams->readDx(mstate2) );
    }
    else
        serr<<"PairInteractionForceField<DataTypes>::addDForce(const MechanicalParams* /*mparams*/ /* PARAMS FIRST */, MultiVecDerivId /*fId*/ ), mstate missing"<<sendl;
}

/*
template<class DataTypes>
void PairInteractionForceField<DataTypes>::addForce(const MechanicalParams* mparams, DataVecDeriv& f1, DataVecDeriv& f2, const DataVecCoord& x1, const DataVecCoord& x2, const DataVecDeriv& v1, const DataVecDeriv& v2 )
{
    addForce( *f1.beginEdit(mparams) , *f2.beginEdit(mparams),
			  x1.getValue(mparams)   , x2.getValue(mparams)  ,
			  v1.getValue(mparams)   , v2.getValue(mparams) );
	f1.endEdit(mparams); f2.endEdit(mparams);
}
template<class DataTypes>
void PairInteractionForceField<DataTypes>::addForce(VecDeriv& , VecDeriv& , const VecCoord& , const VecCoord& , const VecDeriv& , const VecDeriv& )
{
    serr << "ERROR("<<getClassName()<<"): addForce not implemented." << sendl;
}
*/


/*
template<class DataTypes>
void PairInteractionForceField<DataTypes>::addDForce(const MechanicalParams* mparams, DataVecDeriv& df1, DataVecDeriv& df2, const DataVecDeriv& dx1, const DataVecDeriv& dx2)
{
	addDForce(*df1.beginEdit(mparams), *df2.beginEdit(mparams), dx1.getValue(mparams), dx2.getValue(mparams),mparams->kFactor(),mparams->bFactor());
	df1.endEdit(mparams); df2.endEdit(mparams);
}
template<class DataTypes>
void PairInteractionForceField<DataTypes>::addDForce(VecDeriv& df1, VecDeriv& df2, const VecDeriv& dx1, const VecDeriv& dx2, double kFactor, double)
{
    if (kFactor == 1.0)
        addDForce(df1, df2, dx1, dx2);
    else if (kFactor != 0.0)
    {
        VecDerivId vtmp1(VecDerivId::V_FIRST_DYNAMIC_INDEX);
        mstate1->vAvail(vtmp1);
        mstate1->vAlloc(vtmp1);
        VecDerivId vdx1(0);
        /// @TODO: Add a better way to get the current VecId of dx
        for (vdx1.index=0;vdx1.index<vtmp1.index;++vdx1.index)
            if (&mstate1->read(VecDerivId(vdx1))->getValue() == &dx1)
		break;
        VecDeriv* dx1scaled = mstate1->write(vtmp1)->beginEdit();
        dx1scaled->resize(dx1.size());
        mstate1->vOp(vtmp1,VecId::null(),vdx1,kFactor);
        //sout << "dx1 = "<<dx1<<sendl;
        //sout << "dx1*"<<kFactor<<" = "<<dx1scaled<<sendl;
        VecDerivId vtmp2(VecDerivId::V_FIRST_DYNAMIC_INDEX);
        mstate2->vAvail(vtmp2);
        mstate2->vAlloc(vtmp2);
        VecDerivId vdx2(0);
        /// @TODO: Add a better way to get the current VecId of dx
        for (vdx2.index=0;vdx2.index<vtmp2.index;++vdx2.index)

            if (&mstate2->read(VecDerivId(vdx2))->getValue() == &dx2)
		break;
        VecDeriv* dx2scaled = mstate2->write(vtmp2)->beginEdit();
        dx2scaled->resize(dx2.size());
        mstate2->vOp(vtmp2,VecId::null(),vdx2,kFactor);
        //sout << "dx2 = "<<dx2<<sendl;
        //sout << "dx2*"<<kFactor<<" = "<<dx2scaled<<sendl;

        addDForce(df1, df2, *dx1scaled, *dx2scaled);

        mstate1->write(vtmp1)->endEdit();
        mstate2->write(vtmp2)->endEdit();

		mstate1->vFree(vtmp1);
		mstate2->vFree(vtmp2);
    }
}
template<class DataTypes>
void PairInteractionForceField<DataTypes>::addDForce(VecDeriv& , VecDeriv&, const VecDeriv&, const VecDeriv& )
{
    serr << "ERROR("<<getClassName()<<"): addDForce not implemented." << sendl;
}

*/



template<class DataTypes>
double PairInteractionForceField<DataTypes>::getPotentialEnergy(const MechanicalParams* mparams) const
{
    if (mstate1 && mstate2)
        return getPotentialEnergy(mparams /* PARAMS FIRST */, *mparams->readX(mstate1),*mparams->readX(mstate2));
    else return 0.0;
}

/*
template<class DataTypes>
double PairInteractionForceField<DataTypes>::getPotentialEnergy(const MechanicalParams* mparams, const DataVecCoord& x1, const DataVecCoord& x2 ) const
{
	return getPotentialEnergy( x1.getValue(mparams) , x2.getValue(mparams) );
}
template<class DataTypes>
double PairInteractionForceField<DataTypes>::getPotentialEnergy(const VecCoord& , const VecCoord& ) const
{
    serr << "ERROR("<<getClassName()<<"): getPotentialEnergy(const VecCoord1& , const VecCoord2&) not implemented." << sendl;
    return 0.0;
}
*/



template<class DataTypes>
void PairInteractionForceField<DataTypes>::getStatsActivePoints(helper::vector<core::topology::Topology::PointID>& pointsInTopology, sofa::core::topology::BaseMeshTopology* topology, MechanicalState<DataTypes>* mState)
{
    if (!topology || !mState) return;
    pointsInTopology.clear();

    core::topology::TopologyObjectType topologyType = topology->getTopologyType();
    switch (topologyType)
    {
    case core::topology::TopologyObjectType::HEXAHEDRON:
    {
        for (int pid = 0; pid < mState->getSize(); ++pid)
        {
            if (!topology->getHexahedraAroundVertex(pid).empty())
            {
                pointsInTopology.push_back(pid);
            }
        }
        break;
    }
    case core::topology::TopologyObjectType::TETRAHEDRON:
    {
        for (int pid = 0; pid < mState->getSize(); ++pid)
        {
            if (!topology->getTetrahedraAroundVertex(pid).empty())
            {
                pointsInTopology.push_back(pid);
            }
        }
        break;
    }
    case core::topology::TopologyObjectType::QUAD:
    {
        for (int pid = 0; pid < mState->getSize(); ++pid)
        {
            if (!topology->getQuadsAroundVertex(pid).empty())
            {
                pointsInTopology.push_back(pid);
            }
        }
        break;
    }
    case core::topology::TopologyObjectType::TRIANGLE:
    {
        for (int pid = 0; pid < mState->getSize(); ++pid)
        {
            if (!topology->getTrianglesAroundVertex(pid).empty())
            {
                pointsInTopology.push_back(pid);
            }
        }
        break;
    }
    case core::topology::TopologyObjectType::EDGE:
    {
        for (int pid = 0; pid < mState->getSize(); ++pid)
        {
            if (!topology->getEdgesAroundVertex(pid).empty())
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
void PairInteractionForceField<DataTypes>::computeStatsOnAddForce(const unsigned int objectID)
{
    sofa::helper::ReadAccessor< DataVecDeriv > statsAddForcesObj1 = d_statsAddForcesObj1;
    sofa::helper::ReadAccessor< DataVecDeriv > statsAddForcesObj2 = d_statsAddForcesObj2;

    if (objectID == 1u)
    {
        d_statsMaxAddForceObj1.setValue(Real(0));
        d_statsMeanAddForceObj1.setValue(Real(0));
        d_statsMedianAddForceObj1.setValue(Real(0));
        d_statsMinAddForceObj1.setValue(Real(0));

        d_statsNumberOfActiveDofsObj1.setValue(0u);
    }
    else if (objectID == 2u)
    {
        d_statsMaxAddForceObj2.setValue(Real(0));
        d_statsMeanAddForceObj2.setValue(Real(0));
        d_statsMedianAddForceObj2.setValue(Real(0));
        d_statsMinAddForceObj2.setValue(Real(0));

        d_statsNumberOfActiveDofsObj2.setValue(0u);
    }

    if (statsAddForcesObj1.empty() && statsAddForcesObj2.empty()) return;

    using PointID = core::topology::Topology::PointID;

    helper::vector<PointID> pointsInTopology;
    helper::vector<Real> addForcesN;

    if (objectID == 1u)
    {
        getStatsActivePoints(pointsInTopology, m_statsTopology1, mstate1);

        // normalize each element of the addForces vector
        if (!pointsInTopology.empty())
        {
            addForcesN.reserve(pointsInTopology.size());
            for (PointID pid : pointsInTopology)
            {
                addForcesN.push_back(statsAddForcesObj1[pid].norm());
            }
        }
        else
        {
            addForcesN.reserve(statsAddForcesObj1.size());
            for (const Deriv& addForce : statsAddForcesObj1)
            {
                addForcesN.push_back(addForce.norm());
            }
        }
    }
    else if (objectID == 2u)
    {
        getStatsActivePoints(pointsInTopology, m_statsTopology2, mstate2);

        // normalize each element of the addForces vector
        if (!pointsInTopology.empty())
        {
            addForcesN.reserve(pointsInTopology.size());
            for (PointID pid : pointsInTopology)
            {
                addForcesN.push_back(statsAddForcesObj2[pid].norm());
            }
        }
        else
        {
            addForcesN.reserve(statsAddForcesObj2.size());
            for (const Deriv& addForce : statsAddForcesObj2)
            {
                addForcesN.push_back(addForce.norm());
            }
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
    const Real mean = (!addForcesN.empty()) ? std::accumulate(addForcesN.begin(), addForcesN.end(), Real(0)) / addForcesN.size() : Real(0);
    const Real min = addForcesN[0];

    if (objectID == 1u)
    {
        d_statsMaxAddForceObj1.setValue(max);
        d_statsMeanAddForceObj1.setValue(mean);
        d_statsMedianAddForceObj1.setValue(median);
        d_statsMinAddForceObj1.setValue(min);

        d_statsNumberOfActiveDofsObj1.setValue(addForcesN.size());
    }
    else if (objectID == 2u)
    {
        d_statsMaxAddForceObj2.setValue(max);
        d_statsMeanAddForceObj2.setValue(mean);
        d_statsMedianAddForceObj2.setValue(median);
        d_statsMinAddForceObj2.setValue(min);

        d_statsNumberOfActiveDofsObj2.setValue(addForcesN.size());
    }
}

template<class DataTypes>
void PairInteractionForceField<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (!mstate1 || !mstate2 || (!d_drawStatsForcesObj1.getValue() && !d_drawStatsForcesObj2.getValue()
        && !d_drawStatsActiveDofsObj1.getValue() && !d_drawStatsActiveDofsObj2.getValue())) return;

    using PointID = core::topology::Topology::PointID;

    helper::ReadAccessor<Data<VecCoord> > x1 = mstate1->readPositions();
    helper::ReadAccessor<Data<VecCoord> > x2 = mstate2->readPositions();
    sofa::helper::ReadAccessor< DataVecDeriv > statsAddForcesObj1 = d_statsAddForcesObj1;
    sofa::helper::ReadAccessor< DataVecDeriv > statsAddForcesObj2 = d_statsAddForcesObj2;

    if (d_drawStatsForcesObj1.getValue() && statsAddForcesObj1.size() == x1.size())
    {
        float drawStatsArrowWidth = d_drawStatsArrowWidth.getValue();
        float drawStatsArrowScaleLength = d_drawStatsArrowScaleLength.getValue();

        const sofa::defaulttype::Vec4f color = vparams->drawTool()->getColor(d_drawStatsForcesColorObj1.getValue().getSelectedItem());

        for (unsigned int i = 0; i < x1.size(); ++i)
        {
            vparams->drawTool()->drawArrow(defaulttype::Vec3d(DataTypes::getCPos(x1[i])), defaulttype::Vec3d(DataTypes::getCPos(x1[i])) + defaulttype::Vec3d(DataTypes::getDPos(statsAddForcesObj1[i])) * drawStatsArrowScaleLength, drawStatsArrowWidth, color);
        }
    }

    if (d_drawStatsForcesObj2.getValue() && statsAddForcesObj2.size() == x2.size())
    {
        float drawStatsArrowWidth = d_drawStatsArrowWidth.getValue();
        float drawStatsArrowScaleLength = d_drawStatsArrowScaleLength.getValue();

        const sofa::defaulttype::Vec4f color = vparams->drawTool()->getColor(d_drawStatsForcesColorObj2.getValue().getSelectedItem());

        for (unsigned int i = 0; i < x2.size(); ++i)
        {
            vparams->drawTool()->drawArrow(defaulttype::Vec3d(DataTypes::getCPos(x2[i])), defaulttype::Vec3d(DataTypes::getCPos(x2[i])) + defaulttype::Vec3d(DataTypes::getDPos(statsAddForcesObj2[i])) * drawStatsArrowScaleLength, drawStatsArrowWidth, color);
        }
    }

    if (d_drawStatsActiveDofsObj1.getValue())
    {
        helper::vector<PointID> pointsInTopology;
        getStatsActivePoints(pointsInTopology, m_statsTopology1, mstate1);

        sofa::helper::vector<sofa::defaulttype::Vector3> positions;
        float drawStatsSpheresRadius = d_drawStatsSpheresRadius.getValue();

        if (!pointsInTopology.empty())
        {
            for (PointID pid : pointsInTopology)
            {
                positions.push_back(defaulttype::Vec3d(DataTypes::getCPos(x1[pid])));
            }
        }
        else
        {
            for (unsigned int i = 0; i < x1.size(); ++i)
            {
                positions.push_back(defaulttype::Vec3d(DataTypes::getCPos(x1[i])));
            }
        }

        const sofa::defaulttype::Vec4f color = vparams->drawTool()->getColor(d_drawStatsActiveDofsColorObj1.getValue().getSelectedItem());
        vparams->drawTool()->drawSpheres(positions, drawStatsSpheresRadius, color);
    }

    if (d_drawStatsActiveDofsObj2.getValue())
    {
        helper::vector<PointID> pointsInTopology;
        getStatsActivePoints(pointsInTopology, m_statsTopology2, mstate1);

        sofa::helper::vector<sofa::defaulttype::Vector3> positions;
        float drawStatsSpheresRadius = d_drawStatsSpheresRadius.getValue();

        if (!pointsInTopology.empty())
        {
            for (PointID pid : pointsInTopology)
            {
                positions.push_back(defaulttype::Vec3d(DataTypes::getCPos(x2[pid])));
            }
        }
        else
        {
            for (unsigned int i = 0; i < x2.size(); ++i)
            {
                positions.push_back(defaulttype::Vec3d(DataTypes::getCPos(x2[i])));
            }
        }
        const sofa::defaulttype::Vec4f color = vparams->drawTool()->getColor(d_drawStatsActiveDofsColorObj2.getValue().getSelectedItem());
        vparams->drawTool()->drawSpheres(positions, drawStatsSpheresRadius, color);
    }
}


} // namespace behavior

} // namespace core

} // namespace sofa

#endif
