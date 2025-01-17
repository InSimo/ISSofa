/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
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
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_GaussPointContainer_H
#define SOFA_GaussPointContainer_H

#include "../initFlexible.h"
#include "../quadrature/BaseGaussPointSampler.h"


namespace sofa
{
namespace component
{
namespace engine
{

using helper::vector;

/**
 * This class is empty. It is just used to contain custom Gauss points and provide interface with mappings
 */


class SOFA_Flexible_API GaussPointContainer : public BaseGaussPointSampler
{
public:
    typedef BaseGaussPointSampler Inherited;
    SOFA_CLASS(GaussPointContainer,Inherited);

    /** @name  GaussPointSampler types */
    //@{
    typedef Inherited::Real Real;
    typedef Inherited::waVolume waVolume;
    //@}

    Data< unsigned int > f_volumeDim;
    Data< vector<Real> > f_inputVolume;

    virtual std::string getTemplateName() const    { return templateName(this);    }
    static std::string templateName(const GaussPointContainer* = NULL) { return std::string();    }

    virtual void init()
    {
        Inherited::init();
        addInput(&f_inputVolume);
        setDirtyValue();
        update();
    }

    virtual void reinit() { update(); }

protected:
    GaussPointContainer()    :   Inherited()
      , f_volumeDim(initData(&f_volumeDim,(unsigned int)1,"volumeDim","dimension of quadrature weight vectors"))
      , f_inputVolume(initData(&f_inputVolume,"inputVolume","weighted volumes (=quadrature weights)"))
    {

    }

    virtual ~GaussPointContainer()
    {
    }

    virtual void update()
    {
        cleanDirty();

        helper::ReadAccessor< Data< vector<Real> > > invol(f_inputVolume);
        if(!invol.size()) serr<<"no volume provided -> use unit default volume"<<sendl;
        waVolume vol(this->f_volume);
        unsigned int dim = this->f_volumeDim.getValue();

        vol.resize(this->f_position.getValue().size());
        for(unsigned int i=0;i<vol.size();i++)
        {
            vol[i].resize(dim);
            for(unsigned int j=0;j<dim;j++)
            {
                if(!invol.size()) vol[i][j]=(j==0)?1.:0.;
                else if(invol.size()==dim) vol[i][j] = invol[j]; // the same quadrature weights are repeated
                else vol[i][j] = invol[i*dim + j];
            }
        }

    }

};

}
}
}

#endif
