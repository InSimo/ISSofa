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
#ifndef SOFA_COMPONENT_COLLISION_NARROWPHASEDETECTION_H
#define SOFA_COMPONENT_COLLISION_NARROWPHASEDETECTION_H

#include <sofa/core/collision/Detection.h>
#include <sofa/helper/map_ptr_stable_compare.h>
#include <vector>
#include <map>
#include <algorithm>
#include <sofa/core/visual/VisualParams.h>

namespace sofa
{

namespace core
{

namespace collision
{

/**
* @brief Given a set of potentially colliding pairs of models, compute set of contact points
*/

class NarrowPhaseDetection : virtual public Detection
{
public:
    SOFA_ABSTRACT_CLASS_UNIQUE((NarrowPhaseDetection), ((Detection)));

    typedef sofa::helper::map_ptr_stable_compare<std::pair<core::CollisionModel*, core::CollisionModel*>, std::pair<DetectionOutputContainer*, DetectionOutputContainer*>> DetectionOutputMap; // current + previous containers

protected:
    NarrowPhaseDetection()
    :d_showDetectionOutputMap(initData(&d_showDetectionOutputMap, false, "showDetectionOutputMap", "Set to true to draw the content of detection output map"))
    {
    }

    virtual ~NarrowPhaseDetection() { }
public:
    /// Clear all the potentially colliding pairs detected in the previous simulation step
    virtual void beginNarrowPhase()
    {
        // swap output containers and clear the current one
        for (DetectionOutputMap::iterator it = m_outputsMap.begin(); it != m_outputsMap.end(); it++)
        {
            DetectionOutputContainer *currentContainer, *previousContainer;
            std::tie(currentContainer, previousContainer) = it->second;
            currentContainer->swap(*previousContainer);
            if (currentContainer)
            {
                currentContainer->clear();
            }
        }
    }

    /// Add a new potentially colliding pairs of models
    virtual void addCollisionPair (const std::pair<core::CollisionModel*, core::CollisionModel*>& cmPair) = 0;

    /// Add a new list of potentially colliding pairs of models
    virtual void addCollisionPairs(const sofa::helper::vector< std::pair<core::CollisionModel*, core::CollisionModel*> >& v)
    {
        for (sofa::helper::vector< std::pair<core::CollisionModel*, core::CollisionModel*> >::const_iterator it = v.begin(); it!=v.end(); it++)
            addCollisionPair(*it);
    }

    virtual void endNarrowPhase()
    {
        DetectionOutputMap::iterator it = m_outputsMap.begin();
        
        while (it != m_outputsMap.end())
        {
            DetectionOutputContainer *currentContainer = it->second.first;

            if (!currentContainer || currentContainer->empty())
            {
                /// @todo Optimization
                DetectionOutputMap::iterator iterase = it;
				++it;
				m_outputsMap.erase(iterase);
                if (currentContainer) currentContainer->release();
            }
            else
            {
                ++it;
            }
        }
    }

    const DetectionOutputMap& getDetectionOutputs()
    {
        return m_outputsMap;
    }

    std::pair<DetectionOutputContainer*, DetectionOutputContainer*>& getDetectionOutputs(CollisionModel *cm1, CollisionModel *cm2)
    {
        std::pair< CollisionModel*, CollisionModel* > cm_pair = std::make_pair(cm1, cm2);

        DetectionOutputMap::iterator it = m_outputsMap.find(cm_pair);

        if (it == m_outputsMap.end())
        {
            // new contact
            it = m_outputsMap.emplace(cm_pair, std::make_pair(nullptr, nullptr)).first;
        }

        return it->second;
    }

    void draw(const sofa::core::visual::VisualParams* vparams) override
    {
        if (d_showDetectionOutputMap.getValue(vparams))
        {
           
            for (auto it = m_outputsMap.cbegin(); it != m_outputsMap.cend(); ++it)
            {
                const DetectionOutputContainer *currentContainer = it->second.first;
                std::vector< sofa::defaulttype::Vector3 > lines;
                for (std::size_t i=0; i< currentContainer->size(); ++i)
                {
                    DetectionOutput o;
                    if (currentContainer->getDetectionOutput(i, o))
                    {
                        lines.push_back(o.point[0]);
                        lines.push_back(o.point[1]);
                    }
                }

                vparams->drawTool()->drawLines(lines, 1.f, sofa::defaulttype::Vec4f(1.f, 1.f, 1.f, 1.f));
            }
        }

    }

    sofa::Data<bool> d_showDetectionOutputMap;

protected:
    virtual void changeInstanceNP(Instance inst) override
    {
        m_storedOutputsMap[instance].swap(m_outputsMap);
        m_outputsMap.swap(m_storedOutputsMap[inst]);
    }

private:
    std::map<Instance, DetectionOutputMap> m_storedOutputsMap;

    DetectionOutputMap m_outputsMap;
};

} // namespace collision

} // namespace core

} // namespace sofa

#endif
