#include "PythonMacros.h"

#include <sofa/core/objectmodel/Base.h>
#include <sofa/core/objectmodel/BaseContext.h>
#include <sofa/core/objectmodel/Context.h>
//#include <sofa/simulation/tree/GNode.h>
#include <sofa/simulation/common/Node.h>
#include <sofa/core/BaseState.h>
#include <sofa/core/behavior/BaseMechanicalState.h>
#include <sofa/core/loader/BaseLoader.h>
#include <sofa/core/loader/MeshLoader.h>
#include <sofa/core/topology/Topology.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <SofaBaseTopology/MeshTopology.h>
#include <SofaBaseTopology/GridTopology.h>
#include <SofaBaseTopology/RegularGridTopology.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <SofaBaseTopology/PointSetTopologyModifier.h>
#include <SofaMiscMapping/SubsetMultiMapping.h>
#include <SofaBaseTopology/TriangleSetTopologyModifier.h>
#include <sofa/core/BaseMapping.h>
#include "PythonScriptController.h"

#include "Binding_Base.h"
#include "Binding_BaseObject.h"
#include "Binding_BaseState.h"
#include "Binding_BaseMechanicalState.h"
#include "Binding_MechanicalObject.h"
#include "Binding_BaseContext.h"
#include "Binding_Context.h"
#include "Binding_Node.h"
#include "Binding_BaseLoader.h"
#include "Binding_MeshLoader.h"
#include "Binding_Topology.h"
#include "Binding_BaseMeshTopology.h"
#include "Binding_MeshTopology.h"
#include "Binding_GridTopology.h"
#include "Binding_RegularGridTopology.h"
#include "Binding_PythonScriptController.h"
#include "Binding_BaseMapping.h"
//#include "Binding_Mapping.h"
//#include "Binding_RigidMapping.h"
//#include "Binding_MultiMapping.h"
#include "Binding_SubsetMultiMapping.h"
#include "Binding_VisualModel.h"
#include "Binding_PointSetTopologyModifier.h"
#include "Binding_TriangleSetTopologyModifier.h"

typedef sofa::component::container::MechanicalObject< sofa::defaulttype::Vec3Types > MechanicalObject3;
typedef sofa::component::mapping::SubsetMultiMapping< sofa::defaulttype::Vec3Types, sofa::defaulttype::Vec3Types > SubsetMultiMapping3_to_3;


// crée un objet Python à partir d'un objet Cpp héritant de Base,
// retournant automatiquement le type Python de plus haut niveau possible
// en fonction du type de l'objet Cpp
// Ceci afin de permettre l'utilisation de fonctions des sous-classes de Base
PyObject* SP_BUILD_PYSPTR(Base* obj)
{
    if (sofa::simulation::Node::DynamicCast(obj))
        return BuildPySPtr<Base>(obj,&SP_SOFAPYTYPEOBJECT(Node));
    if (Context::DynamicCast(obj))
        return BuildPySPtr<Base>(obj,&SP_SOFAPYTYPEOBJECT(Context));
    if (BaseContext::DynamicCast(obj))
        return BuildPySPtr<Base>(obj,&SP_SOFAPYTYPEOBJECT(BaseContext));

    if (sofa::core::loader::MeshLoader::DynamicCast(obj))
        return BuildPySPtr<Base>(obj,&SP_SOFAPYTYPEOBJECT(MeshLoader));
    if (sofa::core::loader::BaseLoader::DynamicCast(obj))
        return BuildPySPtr<Base>(obj,&SP_SOFAPYTYPEOBJECT(BaseLoader));

    if (sofa::component::topology::RegularGridTopology::DynamicCast(obj))
        return BuildPySPtr<Base>(obj,&SP_SOFAPYTYPEOBJECT(RegularGridTopology));
    if (sofa::component::topology::GridTopology::DynamicCast(obj))
        return BuildPySPtr<Base>(obj,&SP_SOFAPYTYPEOBJECT(GridTopology));
    if (sofa::component::topology::MeshTopology::DynamicCast(obj))
        return BuildPySPtr<Base>(obj,&SP_SOFAPYTYPEOBJECT(MeshTopology));
    if (sofa::component::topology::BaseMeshTopology::DynamicCast(obj))
        return BuildPySPtr<Base>(obj,&SP_SOFAPYTYPEOBJECT(BaseMeshTopology));
    if (sofa::component::topology::Topology::DynamicCast(obj))
        return BuildPySPtr<Base>(obj,&SP_SOFAPYTYPEOBJECT(Topology));

    if (sofa::component::visualmodel::VisualModelImpl::DynamicCast(obj))
        return BuildPySPtr<Base>(obj,&SP_SOFAPYTYPEOBJECT(VisualModel));
    if (MechanicalObject3::DynamicCast(obj))
        return BuildPySPtr<Base>(obj,&SP_SOFAPYTYPEOBJECT(MechanicalObject));
    if (sofa::core::behavior::BaseMechanicalState::DynamicCast(obj))
        return BuildPySPtr<Base>(obj,&SP_SOFAPYTYPEOBJECT(BaseMechanicalState));
    if (sofa::core::BaseState::DynamicCast(obj))
        return BuildPySPtr<Base>(obj,&SP_SOFAPYTYPEOBJECT(BaseState));

    if (sofa::component::controller::PythonScriptController::DynamicCast(obj))
        return BuildPySPtr<Base>(obj,&SP_SOFAPYTYPEOBJECT(PythonScriptController));

    if (SubsetMultiMapping3_to_3::DynamicCast(obj))
        return BuildPySPtr<Base>(obj,&SP_SOFAPYTYPEOBJECT(SubsetMultiMapping3_to_3));
    if (sofa::core::BaseMapping::DynamicCast(obj))
        return BuildPySPtr<Base>(obj,&SP_SOFAPYTYPEOBJECT(BaseMapping));

    if( sofa::component::topology::TriangleSetTopologyModifier::DynamicCast(obj) )
        return BuildPySPtr<Base>(obj,&SP_SOFAPYTYPEOBJECT(TriangleSetTopologyModifier));
    if( sofa::component::topology::PointSetTopologyModifier::DynamicCast(obj) )
        return BuildPySPtr<Base>(obj,&SP_SOFAPYTYPEOBJECT(PointSetTopologyModifier));

    if (BaseObject::DynamicCast(obj))
        return BuildPySPtr<Base>(obj,&SP_SOFAPYTYPEOBJECT(BaseObject));



    // par défaut...
    return BuildPySPtr<Base>(obj,&SP_SOFAPYTYPEOBJECT(Base));
}



void printPythonExceptions()
{
    PyObject *ptype, *pvalue /* error msg */, *ptraceback /*stack snapshot and many other informations (see python traceback structure)*/;
    PyErr_Fetch(&ptype, &pvalue, &ptraceback);
    if( pvalue ) SP_MESSAGE_EXCEPTION( PyString_AsString(pvalue) )

    // TODO improve the error message by using ptraceback
}
