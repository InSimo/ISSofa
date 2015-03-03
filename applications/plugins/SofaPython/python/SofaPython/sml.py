import Sofa

import os.path
import math
import xml.etree.ElementTree as etree 

import Quaternion
import Tools
from Tools import listToStr as concat
import units

def parseIdName(obj,objXml):
    """ set id and name of obj
    """
    obj.id = objXml.attrib["id"]
    obj.name = obj.id
    if not objXml.find("name") is None:
        obj.name = objXml.find("name").text

def parseTag(obj, objXml):
    """ set tags of the object
    """
    obj.tags=set()
    for xmlTag in objXml.iter("tag"):
        obj.tags.add(xmlTag.text)

def parseData(xmlData):
    """ return the list of data in xmlData
    """
    if xmlData.attrib["type"]=="float":
        return Tools.strToListFloat(xmlData.text)
    elif xmlData.attrib["type"]=="int":
        return Tools.strToListInt(xmlData.text)
    elif xmlData.attrib["type"]=="string":
        return xmlData.text.split()

class Model:

    class Mesh:

        class Group:
            def __init__(self):
                self.index=list()
                self.data=dict()
        
        def __init__(self, meshXml):
            self.format = meshXml.find("source").attrib["format"]
            self.source = meshXml.find("source").text
        
            self.group=dict()
            self.data=dict()
            for g in meshXml.iter("group"):
                self.group[g.attrib["id"]] = Model.Mesh.Group()
                self.group[g.attrib["id"]].index = Tools.strToListInt(g.find("index").text)
                for d in g.iter("data"):
                    self.group[g.attrib["id"]].data[d.attrib["name"]]=parseData(d)                    
    
    class Object:
        def __init__(self, objXml):
            parseIdName(self,objXml)
            parseTag(self,objXml)
            self.position=Tools.strToListFloat(objXml.find("position").text)
            self.mesh = None
            self.density=None
            self.mass=None
            if not objXml.find("density") is None:
                self.density=float(objXml.find("density").text)
            if not objXml.find("mass") is None:
                self.mass = float(objXml.find("mass").text)
            self.skinnings=list()

    class Offset:
        def __init__(self, offsetXml):
            self.name = "offset"
            self.value = Tools.strToListFloat(offsetXml.text)
            self.type = offsetXml.attrib["type"]
            
        def isAbsolute(self):
            return self.type == "absolute"
            
    class Dof:
        def __init__(self, dofXml):
            self.index = Model.dofIndex[dofXml.attrib["index"]]
            self.min = None
            self.max = None
            if "min" in dofXml.attrib:
                self.min = float(dofXml.attrib["min"])
            if "max" in dofXml.attrib:
                self.max = float(dofXml.attrib["max"])
        
    class JointGeneric:
        #def __init__(self, name="Unknown",object1,offset1,object2,offset2):
        def __init__(self, jointXml):
            parseIdName(self,jointXml)
            self.objects = [None,None]
            # offsets
            self.offsets = [None,None]
            objects = jointXml.findall("jointObject")
            for i in range(0,2):
                if not objects[i].find("offset") is None:
                    self.offsets[i] = Model.Offset(objects[i].find("offset"))
                    self.offsets[i].name = "offset_{0}".format(self.name)
                    
            # dofs
            self.dofs = [] 
            for dof in jointXml.iter("dof"):
                self.dofs.append(Model.Dof(dof))
        
        
    class Skinning:
        """ Skinning definition, vertices index influenced by bone with weight
        """
        def __init__(self):
            self.object=None
            self.index=list()
            self.weight=list()

    class Surface:
        def __init__(self):
            self.object=None
            self.mesh=None
            self.index=None
            
    class ContactSliding:
        def __init__(self,contactXml):
            parseIdName(self,contactXml)
            self.surfaces = [None,None]
            self.distance=None
            if contactXml.find("distance"):
                self.distance=float(contactXml.findText("distance"))
    
    dofIndex={"x":0,"y":1,"z":2,"rx":3,"ry":4,"rz":5}
    
    def __init__(self, filename, name=None):
        self.name=name
        self.modelDir = os.path.dirname(filename)
        self.units=dict()
        self.meshes=dict()
        self.objects=dict()
        self.objectsByTag=dict()
        self.genericJoints=dict()
        self.slidingContacts=dict()
        
        with open(filename,'r') as f:
            # TODO automatic DTD validation could go here, not available in python builtin ElementTree module
            modelXml = etree.parse(f).getroot()
            if self.name is None and "name" in modelXml.attrib:
                self.name = modelXml.attrib["name"]
            else:
                self.name = os.path.basename(filename)

            # units
            self.parseUnits(modelXml)
            
            # meshes
            for m in modelXml.iter("mesh"):
                if not m.find("source") is None:
                    if m.attrib["id"] in self.meshes:
                        print "WARNING: sml.Model: mesh id {0} already defined".format(m.attrib["id"])
                    mesh = Model.Mesh(m)
                    sourceFullPath = os.path.join(self.modelDir,mesh.source)
                    if os.path.exists(sourceFullPath):
                        mesh.source=sourceFullPath
                    else:
                        print "WARNING: sml.Model: mesh not found:", mesh.source
                    self.meshes[m.attrib["id"]] = mesh
                    
            # objects
            for objXml in modelXml.iter("object"):
                if objXml.attrib["id"] in self.objects:
                    print "ERROR: sml.Model: object defined twice, id:", r.attrib["id"]
                    continue
                object=Model.Object(objXml)
                self.parseMesh(object, objXml)

                mesh=object.mesh # shortcut

                for s in objXml.iter("skinning"):
                    if not s.attrib["object"] in self.objects:
                        print "ERROR: sml.Model: skinning for object {0}: object {1} is not defined".format(name, s.attrib["object"])
                        continue
                    skinning = Model.Skinning()
                    skinning.object = self.objects[s.attrib["object"]]
                    if not (s.attrib["group"] in mesh.group and s.attrib["weight"] in mesh.group[s.attrib["group"]].data):
                        print "ERROR: sml.Model: skinning for object {0}: group {1} - weight {2} is not defined".format(name, s.attrib["group"], s.attrib["weight"])
                        continue
                    skinning.index = mesh.group[s.attrib["group"]].index
                    skinning.weight = mesh.group[s.attrib["group"]].data[s.attrib["weight"]]
                    object.skinnings.append(skinning)

                self.objects[object.id]=object
                for tag in object.tags:
                    if not tag in self.objectsByTag:
                        self.objectsByTag[tag]=list()
                    self.objectsByTag[tag].append(object)
            
            # joints
            self.parseJointGenerics(modelXml)
                
            # contacts
            for c in modelXml.iter("contactSliding"):
                if c.attrib["id"] in self.slidingContacts:
                    print "ERROR: sml.Model: contactSliding defined twice, id:", c.attrib["id"]
                    continue
                contact = Model.ContactSliding(c)
                surfaces=c.findall("surface")
                for i,s in enumerate(surfaces):
                    contact.surfaces[i] = Model.Surface()
                    if s.attrib["object"] in self.objects:
                        contact.surfaces[i].object = self.objects[s.attrib["object"]]
                    else:
                        print "ERROR: sml.Model: in contact {0}, unknown object {1} referenced".format(contact.name, s.attrib["object"])
                    contact.surfaces[i].mesh = contact.surfaces[i].object.mesh # for now a single mesh is supported
                    contact.surfaces[i].index = contact.surfaces[i].mesh.group[s.attrib["group"]].index
                self.slidingContacts[contact.id]=contact
                    

    def parseUnits(self, modelXml):
        xmlUnits = modelXml.find("units")
        if not xmlUnits is None:
            for u in xmlUnits.attrib:
                self.units[u]=xmlUnits.attrib[u]
                
    def parseMesh(self, obj, objXml):
        if not objXml.find("mesh") is None:
            meshId = objXml.find("mesh").attrib["id"]
            if meshId in self.meshes:
                obj.mesh = self.meshes[meshId]
            else:
                print "ERROR: sml.Model: object {0} references undefined mesh {1}".format(obj.name, meshId)

    def parseJointGenerics(self,modelXml):
        for j in modelXml.iter("jointGeneric"):
            if j.attrib["id"] in self.genericJoints:
                print "ERROR: sml.Model: jointGeneric defined twice, id:", j.attrib["id"]
                continue

            joint=Model.JointGeneric(j)
            objects=j.findall("jointObject")
            for i,o in enumerate(objects):
                if o.attrib["id"] in self.objects:
                    joint.objects[i] = self.objects[o.attrib["id"]]
                else:
                    print "ERROR: sml.Model: in joint {0}, unknown object {1} referenced".format(joint.name, o.attrib["id"])
            self.genericJoints[joint.id]=joint
            
def insertVisual(parentNode,obj,color):
    node = parentNode.createChild("node_"+obj.name)
    translation=obj.position[:3]
    rotation = Quaternion.to_euler(obj.position[3:])  * 180.0 / math.pi
    Tools.meshLoader(node, obj.mesh.source, name="loader_"+obj.name, translation=concat(translation),rotation=concat(rotation))
    node.createObject("OglModel",src="@loader_"+obj.name, color=color)
    
def setupUnits(myUnits):
    message = "units:"
    for quantity,unit in myUnits.iteritems():
        exec("units.local_{0} = units.{0}_{1}".format(quantity,unit))
        message+=" "+quantity+":"+unit
    print message

class BaseScene:
    """ Base class for Scene class, creates a node for this Scene
    """
    class Param:
        pass

    def __init__(self,parentNode,model,name=None):
        self.model=model
        self.param=BaseScene.Param()
        self.nodes = dict() # to store special nodes
        n=name
        if n is None:
            n=self.model.name
        self.node=parentNode.createChild(self.model.name)

class SceneDisplay(BaseScene):
    """ Creates a scene to display object meshes
    """
    def __init__(self,parentNode,model):
        BaseScene.__init__(self,parentNode,model)
        self.param.colorDefault="1. 1. 1."
        self.param.colorByTag=dict()
   
    def getTagColor(self, tags):
        """ get the color from the given tags
        if several tags are defined, which corresponds to several colors, one of these color is returned
        """
        tag = tags & set(self.param.colorByTag.keys())
        if len(tag)==0:
            return self.param.colorDefault
        else:
            return self.param.colorByTag[tag.pop()]


    def createScene(self):
        model=self.model # shortcut
        for object in model.objects.values():
            print "Display object:", object.name
            color = self.getTagColor(object.tags)
            insertVisual(self.node, object, color)