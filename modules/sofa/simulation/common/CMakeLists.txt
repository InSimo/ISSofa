cmake_minimum_required(VERSION 2.8)
project(SofaSimulationCommon)

set(HEADER_FILES
	Node.h
    AccumulateTasks.h
	AnimateBeginEvent.h
	AnimateEndEvent.h
	AnimateVisitor.h   
	BehaviorUpdatePositionVisitor.h
	CactusStackStorage.h  
	ClassSystem.h  
	CleanupVisitor.h
	CollisionAnimationLoop.h
	CollisionBeginEvent.h
	CollisionEndEvent.h
	CollisionVisitor.h  
	Colors.h
	CopyAspectVisitor.h
	DeactivatedNodeVisitor.h
	DefaultAnimationLoop.h
	DefaultVisualManagerLoop.h
	DeleteVisitor.h
    DependencyTask.h
	ExportGnuplotVisitor.h
	ExportOBJVisitor.h
	FindByTypeVisitor.h
	BaseGUI.h
	GUIFactory.h
	InitVisitor.h  
	IntegrateBeginEvent.h
	IntegrateEndEvent.h
	LocalStorage.h
    MappingTasks.h
    MechanicalAccumulateMatrixDerivMT.h
	MechanicalComputeEnergyVisitor.h
    MechanicalProjectPositionAndVelocityVisitorMT.h
    MechanicalPropagatePositionAndVelocityVisitorMT.h
    MechanicalPropagatePositionAndVelocityVisitorMT.inl
	MechanicalMatrixVisitor.h
	MechanicalOperations.h
    MechanicalResetConstraintVisitorMT.h
    MechanicalTaskDependencyGraph.h
	MechanicalVPrintVisitor.h
	MechanicalVisitor.h
	MutationListener.h
	ParallelVisitorScheduler.h
	PauseEvent.h
	PipelineImpl.h
	PositionEvent.h
	PrintVisitor.h
    ProjectiveConstraintTasks.h
	PropagateEventVisitor.h
	ReleaseAspectVisitor.h
	ResetVisitor.h
	Simulation.h
    SofaDependencyTask.h
	SolveVisitor.h
	StateChangeVisitor.h
    TaskDependencyGraph.h
    TaskDependencyGraph.inl
    Tasks.h
    Tasks.inl
    TaskScheduler.h
    TaskSchedulerHelpers.h
    TaskStatus.h
    TaskTraits.h
    TopologyChangeEvents.h
	TopologyChangeVisitor.h
	TransformationVisitor.h
	UpdateBoundingBoxVisitor.h
	UpdateContextVisitor.h
	UpdateLinksVisitor.h
	UpdateMappingEndEvent.h
	UpdateMappingVisitor.h
	VectorOperations.h
	VelocityThresholdVisitor.h
	Visitor.h
	VisitorExecuteFunc.h
	VisitorScheduler.h
	VisualVisitor.h
	WriteStateVisitor.h
	XMLPrintVisitor.h
	xml/BaseElement.h
	xml/ElementNameHelper.h
	xml/Element.h
	xml/Element.inl
	xml/initXml.h
	xml/NodeElement.h
	xml/ObjectElement.h
	xml/BaseMultiMappingElement.h
	xml/AttributeElement.h
	xml/DataElement.h
	xml/XML.h   
	SceneLoaderXML.h
	SceneLoaderPHP.h
	SceneLoaderFactory.h
	ScriptEvent.h
	)

set(SOURCE_FILES

	Node.cpp
    AccumulateTasks.cpp
	AnimateBeginEvent.cpp
	AnimateEndEvent.cpp
	AnimateVisitor.cpp
	BehaviorUpdatePositionVisitor.cpp
	CactusStackStorage.cpp
	CleanupVisitor.cpp
	CollisionAnimationLoop.cpp
	CollisionBeginEvent.cpp
	CollisionEndEvent.cpp
	CollisionVisitor.cpp
	CopyAspectVisitor.cpp
	DeactivatedNodeVisitor.cpp
	DefaultAnimationLoop.cpp
	DefaultVisualManagerLoop.cpp
	DeleteVisitor.cpp
    DependencyTask.cpp
	ExportGnuplotVisitor.cpp
	ExportOBJVisitor.cpp
	BaseGUI.cpp
	GUIFactory.cpp
	InitVisitor.cpp   
	IntegrateBeginEvent.cpp
	IntegrateEndEvent.cpp
    MappingTasks.cpp
    MechanicalAccumulateMatrixDerivMT.cpp
	MechanicalComputeEnergyVisitor.cpp
    MechanicalProjectPositionAndVelocityVisitorMT.cpp
    MechanicalPropagatePositionAndVelocityVisitorMT.cpp
	MechanicalOperations.cpp
	MechanicalMatrixVisitor.cpp
    MechanicalResetConstraintVisitorMT.cpp
	MechanicalVPrintVisitor.cpp
	MechanicalVisitor.cpp   
	MutationListener.cpp 
	ParallelVisitorScheduler.cpp
	PauseEvent.cpp
	PipelineImpl.cpp
	PositionEvent.cpp
	PrintVisitor.cpp
    ProjectiveConstraintTasks.cpp
	PropagateEventVisitor.cpp
	ReleaseAspectVisitor.cpp
	ResetVisitor.cpp
	Simulation.cpp  
	SolveVisitor.cpp
    Tasks.cpp
    TaskScheduler.cpp
    TaskSchedulerHelpers.cpp
    TaskStatus.cpp
    TopologyChangeEvents.cpp
	TopologyChangeVisitor.cpp
	StateChangeVisitor.cpp   
	TransformationVisitor.cpp
	UpdateBoundingBoxVisitor.cpp
	UpdateContextVisitor.cpp
	UpdateLinksVisitor.cpp
	UpdateMappingEndEvent.cpp
	UpdateMappingVisitor.cpp
	VectorOperations.cpp
	VelocityThresholdVisitor.cpp
	Visitor.cpp
	VisitorScheduler.cpp
	VisualVisitor.cpp
	WriteStateVisitor.cpp
	XMLPrintVisitor.cpp
	xml/BaseElement.cpp
	xml/ElementNameHelper.cpp
	xml/initXml.cpp
	xml/NodeElement.cpp
	xml/ObjectElement.cpp
	xml/BaseMultiMappingElement.cpp
	xml/AttributeElement.cpp
	xml/DataElement.cpp
	xml/XML.cpp 
	SceneLoaderFactory.cpp
	SceneLoaderXML.cpp
	SceneLoaderPHP.cpp
	ScriptEvent.cpp
	)

if(SOFA-MISC_SMP)
	list(APPEND HEADER_FILES "ChangeListener.h")
	list(APPEND HEADER_FILES "ParallelMechanicalVisitor.h")
	list(APPEND SOURCE_FILES "ChangeListener.cpp")
	list(APPEND SOURCE_FILES "ParallelMechanicalVisitor.cpp")
endif()




include( CMakeDependentOption )
cmake_dependent_option(UNITY_BUILD_${PROJECT_NAME} "Decides if a the generated project files should build through a unity build instead of a normal build. A unity builds packs all .cpp files into a UnityBuild.cpp file and then only include this in the project. This greatly reduces build times." ON  SOFA_ALLOW_UNITY_BUILDS OFF)

if( UNITY_BUILD_${PROJECT_NAME} )
  # Generate a unity build, by creating the UnityBuild.cpp and only including the required 
  # source files.
  set( UNITYBUILD_INCLUDES "" )
  foreach(filename ${SOURCE_FILES})
    SET( UNITYBUILD_INCLUDES "${UNITYBUILD_INCLUDES}#include \"${filename}\"\n" )
  endforeach(filename)

  # Using a cached variable with our string in it. Because cmake doesn't support multi-line strings we have to replace the newlines with a delimiter, so we arbitrarily use +=+.
  string(REPLACE "
" "+=+" UnitybuildIncludesConverted ${UNITYBUILD_INCLUDES}) # Convert the file we're going to write to use our delimiter instead of newlines
  if(NOT (UNITY_BUILD_CACHE_${PROJECT_NAME}) OR NOT (UnitybuildIncludesConverted STREQUAL UNITY_BUILD_CACHE_${PROJECT_NAME})) # If we don't have the cache variable or if its contents don't match our new string then we write the unmodified new UnityBuild file and store the one with the swapped out delimiters in the cache variable
    MESSAGE(STATUS "Updating UnityBuild.cpp for " ${PROJECT_NAME})
    string(REPLACE "
" "+=+" unityBuildCacheNew ${UNITYBUILD_INCLUDES})
    SET(UNITY_BUILD_CACHE_${PROJECT_NAME} ${unityBuildCacheNew} CACHE INTERNAL "Used for determining if UnityBuild.cpp should be updated or not.")
    file(WRITE UnityBuild.cpp ${UNITYBUILD_INCLUDES})
  else()
    MESSAGE(STATUS "Unitybuild.cpp for ${PROJECT_NAME} already up to date")
  endif()
  # overwrite the source files to only include the generated unity build file.
  set( SOURCE_FILES UnityBuild.cpp )
endif()

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

target_link_libraries( ${PROJECT_NAME} PUBLIC ${SofaFramework_LIBRARIES} ) 
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFASIMULATION_SOURCE_INCLUDE_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFASIMULATION_BUILD_INCLUDE_DIR}>"  )
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${SOFASIMULATION_DEPENDENCY_INCLUDE_DIRS}>")

set( SOFASIMULATIONCOMMON_COMPILER_FLAGS "-DSOFA_BUILD_SIMULATION_COMMON")

set_target_properties( ${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d" )
set_target_properties( ${PROJECT_NAME} PROPERTIES VERSION ${SOFASIMULATION_VERSION} )
set_target_properties( ${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${SOFASIMULATIONCOMMON_COMPILER_FLAGS}"  )


install(TARGETS ${PROJECT_NAME}
        COMPONENT SofaSimulation_libraries
        EXPORT  SofaSimulationTargets
        RUNTIME DESTINATION "${SOFASIMULATION_INSTALL_BIN_DIR}"
        LIBRARY DESTINATION "${SOFASIMULATION_INSTALL_LIB_DIR}"
        ARCHIVE DESTINATION "${SOFASIMULATION_INSTALL_LIB_DIR}"
        )
        
if(NOT SOFA_INSTALL_HEADERS_GENERATED_ONLY)
install(DIRECTORY ${SofaSimulationCommon_SOURCE_DIR}/ DESTINATION "${SOFASIMULATION_INSTALL_INCLUDE_DIR}/sofa/simulation/common"
        COMPONENT SofaSimulation_headers
        FILES_MATCHING 
            PATTERN "*.h"
            PATTERN "*.inl"
            PATTERN "*.hxx"
            PATTERN "*.hpp"
        )
endif()

        
        
